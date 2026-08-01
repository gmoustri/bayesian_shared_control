#pragma once

#include <dwal_planner/msg/cluster_group.hpp>
#include <dwal_planner/msg/path_cluster.hpp>
#include <dwal_planner/msg/path.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <vector>

class DWALClusterIntentEstimator
{
public:
  struct Params
  {
    // Prior (sticky + distance-aware redistribution)
    double alpha = 0.8;       // persistence
    double beta  = 2.0;        // exp(-beta*|dphi|) sharpness

    // Likelihood (split-normal around cluster's min-cost phi_C)
    double eta       = 0.5;    // sigma scaling from span
    double sigma_min = 0.03;   // lower bound for sigma

    // Optional gating
    double min_confidence = 0.0;  // set >0 to reject low-confidence selection
  };

  struct Result
  {
    bool valid = false;
    int cluster_index = -1;      // index in msg.clusters
    double confidence = 0.0;     // posterior mass at MAP cluster
    std::vector<double> belief;     // Optional: full posterior belief over clusters (for debugging/analysis)
  };

  explicit DWALClusterIntentEstimator(const Params& p) : p_(p) {}

  void reset()
  {
    belief_.clear();
    prev_ids_.clear();
    prev_phiC_.clear();
  }

  Result update(double phi_user, const dwal_planner::msg::ClusterGroup& msg)
  {
    Result out;

    const size_t Kt = msg.clusters.size();
    if (Kt == 0) return out;

    // --- Extract cluster representative geometry for time t ---
    // We keep:
    //  - id_i
    //  - phi_k_i = min phi in cluster
    //  - phi_l_i = max phi in cluster
    //  - phi_C_i = phi of min-cost path
    std::vector<uint32_t> ids;
    std::vector<double> phi_k, phi_l, phi_C;
    ids.reserve(Kt); phi_k.reserve(Kt); phi_l.reserve(Kt); phi_C.reserve(Kt);

    for (const auto& c : msg.clusters) {
      if (c.paths.empty()) {
        // still keep placeholder to preserve indexing with msg.clusters
        ids.push_back(c.id);
        phi_k.push_back(0.0);
        phi_l.push_back(0.0);
        phi_C.push_back(0.0);
        continue;
      }

      double pk =  std::numeric_limits<double>::infinity();
      double pl = -std::numeric_limits<double>::infinity();
      double pC = 0.0;
      int min_cost = std::numeric_limits<int>::max();

      for (const auto& p : c.paths) {
        pk = std::min(pk, p.phi);
        pl = std::max(pl, p.phi);
        if (p.cost < min_cost) {
          min_cost = p.cost;
          pC = p.phi;
        }
      }

      ids.push_back(c.id);
      phi_k.push_back(pk);
      phi_l.push_back(pl);
      phi_C.push_back(pC);
    }

    // --- Initialize belief uniformly on first valid call ---
    if (belief_.empty() || prev_ids_.empty()) {
      belief_.assign(Kt, 1.0 / std::max<size_t>(1, Kt));
      prev_ids_ = ids;
      prev_phiC_ = phi_C;
      // pick MAP 
      const auto [idx, conf] = argmax(belief_);
      out.valid = true;
      out.cluster_index = static_cast<int>(idx);
      out.confidence = conf;
      if (out.confidence < p_.min_confidence) out = Result{};
      return out;
    }

    // If K changes, we still do the correct thing: prior is defined over current Kt.
    std::vector<double> prior(Kt, 0.0);

    // --- Prior update: sticky + distance-aware transitions ---
    // This matches your paper’s intent:
    // - If prev cluster j matches a current i by id: sticky mass alpha goes to i,
    //   remainder (1-alpha) redistributed by exp(-beta*|phi_C(i)-phi_C_prev(j)|).
    // - If no match: FULL mass redistributed by exp(-beta*distance) (no alpha loss).
    for (size_t j = 0; j < prev_ids_.size(); ++j) {
      const double bj = (j < belief_.size()) ? belief_[j] : 0.0;
      if (bj <= 0.0) continue;

      const uint32_t prev_id = prev_ids_[j];
      const double prev_phiC = prev_phiC_[j];

      // find match by id
      int match = -1;
      for (size_t i = 0; i < Kt; ++i) {
        if (ids[i] == prev_id) { match = static_cast<int>(i); break; }
      }

      const double sticky_mass = (match >= 0) ? p_.alpha : 0.0;
      if (match >= 0) {
        prior[static_cast<size_t>(match)] += sticky_mass * bj;
      }

      const double spread_mass = 1.0 - sticky_mass; // = (1-alpha) if matched, = 1 if unmatched

      double Z = 0.0;
      std::vector<double> w(Kt, 0.0);
      for (size_t i = 0; i < Kt; ++i) {
        // Exclude matched successor from redistribution when match exists
        if (match >= 0 && static_cast<int>(i) == match) {
          w[i] = 0.0;
          continue;
        }
        const double d = std::abs(phi_C[i] - prev_phiC);
        w[i] = std::exp(-p_.beta * d);
        Z += w[i];
      }

      // If match exists, we already placed alpha mass on it.
      // If match does NOT exist, we redistribute FULL mass over all clusters,
      // so in that case we must allow all i (including what would be "match").
      // Here match<0 already means we didn't exclude anything.

      if (Z < 1e-12) {
        // Degenerate case:
        // - if match exists: put all mass on match
        // - else: uniform
        if (match >= 0) {
          prior[static_cast<size_t>(match)] += (1.0 - p_.alpha) * bj; // or just bj if you prefer
        } else {
          const double u = 1.0 / std::max<size_t>(1, Kt);
          for (size_t i = 0; i < Kt; ++i) prior[i] += bj * u;
        }
      } else {
        for (size_t i = 0; i < Kt; ++i) {
          if (match >= 0 && static_cast<int>(i) == match) continue;
          prior[i] += spread_mass * bj * (w[i] / Z);
        }
      }
    }

    normalizeInPlace(prior);

    // --- Likelihood: split-normal around phi_C(i) with sigma from span ---
    std::vector<double> post(Kt, 0.0);

    for (size_t i = 0; i < Kt; ++i) {
      // If cluster has empty paths we gave placeholders; make it effectively impossible
      // unless everything is empty.
      if (!msg.clusters[i].paths.empty()) {
        const double sigmaL = std::max(p_.eta * (phi_C[i] - phi_k[i]), p_.sigma_min);
        const double sigmaR = std::max(p_.eta * (phi_l[i] - phi_C[i]), p_.sigma_min);
        const double d = phi_user - phi_C[i];
        const double sigma = (phi_user <= phi_C[i]) ? sigmaL : sigmaR;
        post[i] = prior[i] * std::exp(-0.5 * d * d / (sigma * sigma));
      } else {
        post[i] = 0.0;
      }
    }

    // If all are zero (e.g., all empty paths), fall back to prior
    if (std::accumulate(post.begin(), post.end(), 0.0) < 1e-12) {
      post = prior;
    } else {
      normalizeInPlace(post);
    }
    
    belief_ = post;
    prev_ids_ = ids;
    prev_phiC_ = phi_C;

    const auto [idx, conf] = argmax(belief_);
    out.valid = true;
    out.cluster_index = static_cast<int>(idx);
    out.confidence = conf;
    out.belief = belief_;

    if (out.confidence < p_.min_confidence) out = Result{};
    return out;
  }
  
private:
  Params p_;
  std::vector<double> belief_;     // posterior b_t over prev set
  std::vector<uint32_t> prev_ids_;
  std::vector<double> prev_phiC_;

  static void normalizeInPlace(std::vector<double>& v)
  {
    double s = std::accumulate(v.begin(), v.end(), 0.0);
    if (s < 1e-12) {
      const double u = 1.0 / std::max<size_t>(1, v.size());
      for (auto& x : v) x = u;
      return;
    }
    for (auto& x : v) x /= s;
  }

  static std::pair<size_t, double> argmax(const std::vector<double>& v)
  {
    if (v.empty()) return {0, 0.0};
    auto it = std::max_element(v.begin(), v.end());
    return {static_cast<size_t>(std::distance(v.begin(), it)), *it};
  }
};