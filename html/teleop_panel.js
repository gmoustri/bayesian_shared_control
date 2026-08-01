// const FIXED_FRAME = "odom";
// const FOLLOW_FRAME = "base_link";
const FOLLOW_HEIGHT = 1.2;   // meters behind/above target
const FOLLOW_BACK = 3.0;
let followEnabled = true;
let odom0 = null;          // {x,y}
let cam0 = null;           // THREE.Vector3
let target0 = null;        // THREE.Vector3
let odomNow = { x: 0, y: 0 };  // ROS x,y
let haveOdom = false;
let lastOdom = null; // {x,y}

let yawNow = 0.0;
let lastYaw = null;
let recentered = false;

let bagStart, bagStop, bagDelete;
let isRecording = false;
let pendingBag = "";


let ros, scene, camera, renderer, controls;
const tfTree = new Map();
const followers = new Map(); // frame_id -> Set(objects)

function strip(s) { return (s || "").replace(/^\//, ''); }

function status(t, ok = false) {
    const s = document.getElementById("status");
    s.textContent = t;
    s.style.background = ok ? "#d7f5d7" : "#eee";
    s.style.border = ok ? "1px solid #5bb85b" : "1px solid #ddd";
}

function connect() {
    ros = new ROSLIB.Ros({ url: document.getElementById("ws").value });
    ros.on("connection", () => { status("CONNECTED", true); init(); });
    ros.on("error", (e) => { console.error(e); status("ERROR"); });
    ros.on("close", () => status("DISCONNECTED"));
}

// ---------- TF ----------
function setTf(parent, child, t, q) {
    tfTree.set(child, { parent, t, q });
}
function quatMat(q, t) {
    const quat = new THREE.Quaternion(q.x, q.y, q.z, q.w);
    const pos = new THREE.Vector3(t.x, t.y, t.z);
    const m = new THREE.Matrix4();
    m.compose(pos, quat, new THREE.Vector3(1, 1, 1));
    return m;
}
// Return matrix that places an object in FIXED_FRAME coordinates
function getFixedTo(frame) {
    frame = strip(frame);
    if (!frame || frame === FIXED_FRAME) return new THREE.Matrix4();

    let cur = frame;
    let m = new THREE.Matrix4().identity();

    for (let i = 0; i < 120; i++) {
        if (cur === FIXED_FRAME) return m.invert(); // critical direction
        const link = tfTree.get(cur);
        if (!link) return null;

        const parentToChild = quatMat(link.q, link.t);
        const childToParent = new THREE.Matrix4().copy(parentToChild).invert();
        m.premultiply(childToParent);
        cur = strip(link.parent);
    }
    return null;
}

function getPositionInFixed(frame) {
    frame = strip(frame);
    if (!frame) return null;

    // Accumulate FIXED_FRAME -> frame
    let cur = frame;
    let chain = [];
    for (let i = 0; i < 120; i++) {
        if (cur === FIXED_FRAME) break;
        const link = tfTree.get(cur);
        if (!link) return null;
        chain.push({ child: cur, parent: strip(link.parent), t: link.t, q: link.q });
        cur = strip(link.parent);
    }
    if (cur !== FIXED_FRAME) return null;
    // Build FIXED->frame by multiplying parent->child along the chain (reverse order)
    let M = new THREE.Matrix4().identity();
    for (let i = chain.length - 1; i >= 0; i--) {
        const parentToChild = quatMat(chain[i].q, chain[i].t);
        M.multiply(parentToChild);
    }

    // Extract ROS position
    const rosP = new THREE.Vector3().setFromMatrixPosition(M);

    // Convert ROS (x,y,z) to your scene coords (x, y=height, z=y)
    return new THREE.Vector3(rosP.x, rosP.z, rosP.y);
}

function followObject(obj, frame) {
    frame = strip(frame || FIXED_FRAME);
    if (!followers.has(frame)) followers.set(frame, new Set());
    followers.get(frame).add(obj);
}

function init() {
    const container = document.getElementById("viewer");

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0.3, 0.3, 0.3);


    camera = new THREE.PerspectiveCamera(60, container.clientWidth / container.clientHeight, 0.01, 200);
    camera.up.set(0, 0, 1);
    camera.position.set(6, 6, 6);
    camera.lookAt(0, 0, 0);


    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(renderer.domElement);

    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.target.set(0, 0, 0);
    controls.update();

    cam0 = camera.position.clone();
    target0 = controls.target.clone();

    window.addEventListener("resize", () => {
        camera.aspect = container.clientWidth / container.clientHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(container.clientWidth, container.clientHeight);
    });


    bagStart = new ROSLIB.Service({
        ros,
        name: "/bag_record_start",
        serviceType: "std_srvs/srv/Trigger"
    });

    bagStop = new ROSLIB.Service({
        ros,
        name: "/bag_record_stop",
        serviceType: "std_srvs/srv/Trigger"
    });

    bagDelete = new ROSLIB.Service({
        ros,
        name: "/bag_record_delete_last",
        serviceType: "std_srvs/srv/Trigger"
    });

    const recordBtn = document.getElementById("recordBtn");
    const keepBtn = document.getElementById("keepBtn");
    const deleteBtn = document.getElementById("deleteBtn");
    const recordStatus = document.getElementById("recordStatus");

    function setRecordIdle() {
        isRecording = false;
        recordBtn.textContent = "Start Recording";
    }

    function hideDecisionButtons() {
        keepBtn.style.display = "none";
        deleteBtn.style.display = "none";
    }

    function showDecisionButtons() {
        keepBtn.style.display = "inline-block";
        deleteBtn.style.display = "inline-block";
    }

    recordBtn.onclick = () => {
        recordBtn.disabled = true;

        if (!isRecording) {
            bagStart.callService(new ROSLIB.ServiceRequest({}), (res) => {
                recordBtn.disabled = false;
                if (res.success) {
                    isRecording = true;
                    pendingBag = "";
                    hideDecisionButtons();
                    recordBtn.textContent = "Stop Recording";
                    recordStatus.textContent = "REC";
                } else {
                    recordStatus.textContent = res.message;
                }
            });
        } else {
            bagStop.callService(new ROSLIB.ServiceRequest({}), (res) => {
                recordBtn.disabled = false;
                if (res.success) {
                    pendingBag = res.message;
                    setRecordIdle();
                    showDecisionButtons();
                    recordStatus.textContent = `Stopped: ${pendingBag}`;
                } else {
                    recordStatus.textContent = res.message;
                }
            });
        }
    };

    keepBtn.onclick = () => {
        pendingBag = "";
        hideDecisionButtons();
        recordStatus.textContent = "Recording kept";
    };

    deleteBtn.onclick = () => {
        deleteBtn.disabled = true;
        bagDelete.callService(new ROSLIB.ServiceRequest({}), (res) => {
            deleteBtn.disabled = false;
            if (res.success) {
                pendingBag = "";
                hideDecisionButtons();
                recordStatus.textContent = "Recording deleted";
            } else {
                recordStatus.textContent = res.message;
            }
        });
    };

    function renderPath(topicName, colorHex, yLift) {
        const group = new THREE.Object3D();
        scene.add(group);

        let pathMesh = null;

        const sub = new ROSLIB.Topic({
            ros,
            name: topicName,
            messageType: "nav_msgs/msg/Path",
            queue_length: 1,
            throttle_rate: 100
        });

        sub.subscribe((msg) => {
            try {
                const poses = msg.poses || [];
                if (poses.length < 2) return;

                const frame = msg.header?.frame_id || "base_link";
                const T = getFixedTo(frame);
                if (!T) return;

                const points = [];
                for (const ps of poses) {
                    const p = ps.pose.position;
                    const v = new THREE.Vector3(p.x, p.y, yLift);
                    v.applyMatrix4(T);
                    points.push(v);
                }

                if (points.length < 2) return;

                const curve = new THREE.CatmullRomCurve3(points);
                const tubeGeom = new THREE.TubeGeometry(
                    curve,
                    Math.max(8, points.length * 2), // tubular segments
                    0.01,                           // radius = thickness
                    8,                              // radial segments
                    false
                );

                const mat = new THREE.MeshBasicMaterial({
                    color: new THREE.Color(colorHex)
                });

                if (pathMesh) {
                    group.remove(pathMesh);
                    pathMesh.geometry.dispose();
                    pathMesh.material.dispose();
                }

                pathMesh = new THREE.Mesh(tubeGeom, mat);
                group.add(pathMesh);

            } catch (e) {
                console.error("renderPath error:", topicName, e);
            }
        });
    }

    // function renderPath(topicName, colorHex, yLift) {
    //     const group = new THREE.Object3D();
    //     scene.add(group);

    //     let line = null;

    //     const sub = new ROSLIB.Topic({
    //         ros,
    //         name: topicName,
    //         messageType: "nav_msgs/msg/Path",
    //         queue_length: 1,
    //         throttle_rate: 100
    //     });

    //     sub.subscribe((msg) => {
    //         try {
    //             const poses = msg.poses || [];
    //             if (poses.length < 2) return;

    //             const frame = msg.header?.frame_id || "base_link";
    //             const T = getFixedTo(frame);
    //             if (!T) return;

    //             const verts = [];
    //             for (const ps of poses) {
    //                 const p = ps.pose.position;
    //                 const v = new THREE.Vector3(p.x, p.y, yLift);
    //                 v.applyMatrix4(T);
    //                 verts.push(v.x, v.y, v.z);
    //             }

    //             const geom = new THREE.BufferGeometry();
    //             geom.setAttribute("position", new THREE.Float32BufferAttribute(verts, 3));

    //             const mat = new THREE.LineBasicMaterial({
    //                 color: new THREE.Color(colorHex)
    //             });

    //             if (line) {
    //                 group.remove(line);
    //                 line.geometry.dispose();
    //                 line.material.dispose();
    //             }

    //             line = new THREE.Line(geom, mat);
    //             group.add(line);

    //         } catch (e) {
    //             console.error("renderPath error:", topicName, e);
    //         }
    //     });
    // }

    renderPath(TOPICS.path_human, 0xff8800, 0.06);
    renderPath(TOPICS.path_shared, 0x00cc00, 0.03);

    // Grid + axes
    //scene.add(new THREE.GridHelper(20,20));
    const grid = new THREE.GridHelper(20, 20);
    grid.rotation.x = Math.PI / 2;   // XZ -> XY
    scene.add(grid);

    // TF topics
    const tfTopic = new ROSLIB.Topic({
        ros,
        name: TOPICS.tf,
        messageType: "tf2_msgs/msg/TFMessage",
        queue_length: 1,
        throttle_rate: 0
    });

    const tfStaticTopic = new ROSLIB.Topic({
        ros,
        name: TOPICS.tf_static,
        messageType: "tf2_msgs/msg/TFMessage",
        queue_length: 1,
        throttle_rate: 0
    });

    function ingest(msg) {
        const arr = msg.transforms || [];
        for (const tr of arr) {
            setTf(strip(tr.header.frame_id),
                strip(tr.child_frame_id),
                tr.transform.translation,
                tr.transform.rotation);
        }
    }
    tfTopic.subscribe(ingest);
    tfStaticTopic.subscribe(ingest);

    // Odom helper (axes at odom pose)
    const odomObj = new THREE.AxesHelper(0.3);
    scene.add(odomObj);
    new ROSLIB.Topic({
        ros,
        name: TOPICS.odom,
        messageType: "nav_msgs/msg/Odometry",
        queue_length: 1,
        throttle_rate: 0
    }).subscribe(msg => {
        const p = msg.pose.pose.position;
        const q = msg.pose.pose.orientation;
        odomNow.x = p.x;
        odomNow.y = p.y;
        haveOdom = true;

        function yawFromQuat(q) {
            // yaw about +Y in your scene (planar)
            const siny = 2 * (q.w * q.z + q.x * q.y);
            const cosy = 1 - 2 * (q.y * q.y + q.z * q.z);
            return Math.atan2(siny, cosy);
        }
        yawNow = yawFromQuat(q);
        if (lastYaw === null) lastYaw = yawNow;

        if (!odom0) odom0 = { x: p.x, y: p.y };
        if (!lastOdom) lastOdom = { x: p.x, y: p.y };
        haveOdom = true;
        // ---- recenter camera once on first odom ----
        if (!recentered) {
            const dx0 = p.x;
            const dz0 = p.y;
            camera.position.add(new THREE.Vector3(dx0, dz0, 0));
            controls.target.add(new THREE.Vector3(dx0, dz0, 0));
            controls.update();
            lastOdom = { x: p.x, y: p.y };
            recentered = true;
        }
        odomObj.position.set(p.x, p.y, p.z);
        odomObj.quaternion.set(q.x, q.y, q.z, q.w);
    });

    // ---------- COSTMAP (manual, stable) ----------
    const costObj = new THREE.Object3D();
    scene.add(costObj);

    const plane = new THREE.Mesh(
        new THREE.PlaneGeometry(1, 1),
        new THREE.MeshBasicMaterial({ transparent: true, opacity: 0.7, side: THREE.DoubleSide })
    );

    costObj.add(plane);

    const canvas = document.createElement("canvas");
    const ctx = canvas.getContext("2d");
    const tex = new THREE.CanvasTexture(canvas);
    // Stop “Texture resized … to power-of-two” warnings:
    tex.flipY = false;
    tex.needsUpdate = true;
    tex.generateMipmaps = false;
    tex.minFilter = THREE.LinearFilter;
    tex.magFilter = THREE.LinearFilter;
    tex.wrapS = THREE.ClampToEdgeWrapping;
    tex.wrapT = THREE.ClampToEdgeWrapping;
    plane.material.map = tex;

    let costFollowSet = false;

    new ROSLIB.Topic({
        ros,
        name: TOPICS.costmap,
        messageType: "nav_msgs/msg/OccupancyGrid",
        queue_length: 1,
        throttle_rate: 0
    }).subscribe(grid => {
        const w = grid.info.width, h = grid.info.height;
        if (!w || !h) return;

        if (canvas.width !== w || canvas.height !== h) {
            canvas.width = w; canvas.height = h;
        }

        const img = ctx.createImageData(w, h);
        const data = grid.data;

        for (let i = 0; i < w * h; i++) {
            const v = data[i];
            const j = i * 4;
            if (v < 0) {
                img.data[j + 3] = 0;
            } else {
                const c = 255 - Math.round(Math.min(100, v) * 2.55);
                img.data[j] = img.data[j + 1] = img.data[j + 2] = c;
                img.data[j + 3] = 255;
            }
        }
        ctx.putImageData(img, 0, 0);
        tex.needsUpdate = true;

        const res = grid.info.resolution;
        plane.scale.set(w * res, h * res, 1);

        const ox = grid.info.origin.position.x;
        const oy = grid.info.origin.position.y;
        plane.position.set(ox + (w * res) / 2, oy + (h * res) / 2, 0.0);

        if (!costFollowSet) {
            followObject(costObj, grid.header.frame_id);
            costFollowSet = true;
        }
    });

    // ---------- FOOTPRINT (PolygonStamped) ----------
    const footprintObj = new THREE.Object3D();
    scene.add(footprintObj);
    let footprintFollowSet = false;

    new ROSLIB.Topic({
        ros,
        name: TOPICS.footprint,
        messageType: "geometry_msgs/msg/PolygonStamped",
        queue_length: 1,
        throttle_rate: 0
    }).subscribe(msg => {
        const pts = msg.polygon?.points || [];
        if (pts.length < 3) return;

        // rebuild line
        while (footprintObj.children.length) footprintObj.remove(footprintObj.children[0]);

        const verts = [];
        for (const p of pts) verts.push(p.x, p.y, 0.02);
        verts.push(pts[0].x, pts[0].y, 0.0);

        const geom = new THREE.BufferGeometry();
        geom.setAttribute("position", new THREE.Float32BufferAttribute(verts, 3));
        footprintObj.add(new THREE.Line(geom, new THREE.LineBasicMaterial()));

        if (!footprintFollowSet) {
            followObject(footprintObj, msg.header.frame_id);
            footprintFollowSet = true;
        }
    });

    // ---------- MARKERS (persistent, supports common types) ----------
    function mkColor(c) {
        const a = (c?.a ?? 1);
        return { color: new THREE.Color(c?.r ?? 1, c?.g ?? 1, c?.b ?? 1), opacity: a };
    }

    function renderMarkerArray(topicName) {
        const group = new THREE.Group();
        scene.add(group);

        const cache = new Map(); // key ns/id -> Object3D
        let followSet = false;

        new ROSLIB.Topic({
            ros,
            name: topicName,
            messageType: "visualization_msgs/msg/MarkerArray",
            queue_length: 1,
            throttle_rate: 0
        }).subscribe(msg => {
            const markers = msg.markers || [];
            if (markers.length && !followSet) {
                followObject(group, markers[0].header?.frame_id || FIXED_FRAME);
                followSet = true;
            }

            for (const m of markers) {
                const key = (m.ns ?? "") + "/" + (m.id ?? 0);

                // action: 0 ADD/MODIFY, 2 DELETE, 3 DELETEALL
                if (m.action === 3) {
                    group.clear();
                    cache.clear();
                    continue;
                }
                if (m.action === 2) {
                    const obj = cache.get(key);
                    if (obj) { group.remove(obj); cache.delete(key); }
                    continue;
                }

                const col = mkColor(m.color);
                const scl = m.scale || { x: 0.1, y: 0.1, z: 0.1 };

                let obj = cache.get(key);

                // helper: set pose
                const setPose = (o) => {
                    const p = m.pose?.position || { x: 0, y: 0, z: 0 };
                    const q = m.pose?.orientation || { x: 0, y: 0, z: 0, w: 1 };
                    o.position.set(p.x, p.y, p.z);
                    o.quaternion.set(q.x, q.y, q.z, q.w);
                };

                // rebuild for dynamic-geometry types
                const rebuild = () => {
                    if (obj) { group.remove(obj); cache.delete(key); }
                    obj = null;
                };

                // types:
                // 1 CUBE, 2 SPHERE, 4 LINE_STRIP, 5 LINE_LIST, 6 CUBE_LIST, 7 SPHERE_LIST, 8 POINTS
                if (m.type === 1 || m.type === 2) {
                    if (!obj) {
                        const geom = (m.type === 1)
                            ? new THREE.BoxGeometry(scl.x, scl.y, scl.z)
                            : new THREE.SphereGeometry(Math.max(scl.x, scl.y, scl.z) / 2, 12, 12);
                        const mat = new THREE.MeshBasicMaterial({ color: col.color, transparent: true, opacity: col.opacity });
                        obj = new THREE.Mesh(geom, mat);
                        group.add(obj); cache.set(key, obj);
                    } else {
                        if (obj.material) {
                            obj.material.color = col.color;
                            obj.material.opacity = col.opacity;
                            obj.material.transparent = true;
                        }
                    }
                    setPose(obj);
                }

                else if (m.type === 4 || m.type === 5) {
                    rebuild();
                    const pts = m.points || [];
                    if (pts.length < 2) continue;

                    const tubeRadius = 0.008;   // thickness
                    const radialSegs = 6;

                    if (m.type === 4) {
                        // LINE_STRIP -> one tube
                        const curvePts = pts.map(p => new THREE.Vector3(p.x, p.y, p.z));
                        const curve = new THREE.CatmullRomCurve3(curvePts);

                        const geom = new THREE.TubeGeometry(
                            curve,
                            Math.max(8, curvePts.length * 2),
                            tubeRadius,
                            radialSegs,
                            false
                        );

                        const mat = new THREE.MeshBasicMaterial({
                            color: col.color,
                            transparent: true,
                            opacity: col.opacity
                        });

                        obj = new THREE.Mesh(geom, mat);
                    } else {
                        // LINE_LIST -> one tube per segment
                        const segGroup = new THREE.Group();

                        for (let i = 0; i + 1 < pts.length; i += 2) {
                            const p0 = pts[i];
                            const p1 = pts[i + 1];

                            const curve = new THREE.LineCurve3(
                                new THREE.Vector3(p0.x, p0.y, p0.z),
                                new THREE.Vector3(p1.x, p1.y, p1.z)
                            );

                            const geom = new THREE.TubeGeometry(
                                curve,
                                2,
                                tubeRadius,
                                radialSegs,
                                false
                            );

                            const mat = new THREE.MeshBasicMaterial({
                                color: col.color,
                                transparent: true,
                                opacity: col.opacity
                            });

                            segGroup.add(new THREE.Mesh(geom, mat));
                        }

                        obj = segGroup;
                    }

                    group.add(obj);
                    cache.set(key, obj);
                    setPose(obj);
                }

                else if (m.type === 8) {
                    // POINTS
                    rebuild();
                    const pts = m.points || [];
                    if (!pts.length) continue;

                    const verts = [];
                    for (const p of pts) verts.push(p.x, p.y, p.z);

                    const geom = new THREE.BufferGeometry();
                    geom.setAttribute("position", new THREE.Float32BufferAttribute(verts, 3));
                    const mat = new THREE.PointsMaterial({
                        size: Math.max(0.01, scl.x || 0.03),
                        color: col.color,
                        transparent: true,
                        opacity: col.opacity
                    });
                    obj = new THREE.Points(geom, mat);
                    group.add(obj); cache.set(key, obj);
                    setPose(obj);
                }

                else if (m.type === 6 || m.type === 7) {
                    // CUBE_LIST / SPHERE_LIST
                    rebuild();
                    const pts = m.points || [];
                    if (!pts.length) continue;

                    const g = new THREE.Group();
                    const mat = new THREE.MeshBasicMaterial({ color: col.color, transparent: true, opacity: col.opacity });

                    for (const p of pts) {
                        const geom = (m.type === 6)
                            ? new THREE.BoxGeometry(scl.x, scl.y, scl.z)
                            : new THREE.SphereGeometry(Math.max(scl.x, scl.y, scl.z) / 2, 10, 10);
                        const mesh = new THREE.Mesh(geom, mat);
                        mesh.position.set(p.x, p.y, p.z);
                        g.add(mesh);
                    }
                    obj = g;
                    group.add(obj); cache.set(key, obj);
                    setPose(obj);
                }

                // else: ignore other marker types for quick demo
            }
        });
    }

    renderMarkerArray(TOPICS.markers_far);
    renderMarkerArray(TOPICS.markers_near);

    // ---------- LOOP ----------
    function animate() {
        requestAnimationFrame(animate);

        // Apply TF to followed objects
        for (const [frame, objs] of followers.entries()) {
            const m = getFixedTo(frame);
            if (!m) continue;
            for (const o of objs) {
                o.matrixAutoUpdate = false;
                o.matrix.copy(m);
            }
        }

        if (followEnabled && haveOdom && lastOdom && lastYaw !== null) {
            const dx = odomNow.x - lastOdom.x;
            const dz = odomNow.y - lastOdom.y; // ROS y -> scene z
            const UP = new THREE.Vector3(0, 0, 1); // since your "up" is the 3rd axis in your mapping

            // incremental yaw (wrap to [-pi, pi])
            let dyaw = yawNow - lastYaw;
            if (dyaw > Math.PI) dyaw -= 2 * Math.PI;
            if (dyaw < -Math.PI) dyaw += 2 * Math.PI;
            if (Math.abs(dyaw) > 1e-6) {
                // rotate camera around the robot (use current target as pivot)
                camera.position.sub(controls.target).applyAxisAngle(UP, dyaw).add(controls.target);
                // (optional) if you want the view direction to remain exactly constant in robot frame:
                // controls.target.sub(controls.target).applyAxisAngle(UP, dyaw).add(controls.target); // no-op, skip
            }

            lastYaw = yawNow;

            // shift the *current* camera and target, preserving mouse edits
            camera.position.add(new THREE.Vector3(dx, dz, 0));
            controls.target.add(new THREE.Vector3(dx, dz, 0));

            lastOdom.x = odomNow.x;
            lastOdom.y = odomNow.y;
        }
        controls.update();
        renderer.render(scene, camera);
    }


    let teleopEnabled = false;
    const cmdVel = new ROSLIB.Topic({
        ros,
        name: "/cmd_vel_web",
        messageType: "geometry_msgs/msg/Twist",
        queue_size: 1
    });

    const joyTopic = new ROSLIB.Topic({
        ros,
        name: "/joy_web",
        messageType: "sensor_msgs/msg/Joy",
        queue_size: 1
    });

    function dz(x, d = 0.1) {
        return Math.abs(x) < d ? 0.0 : x;
    }

    function sendZero() {
        cmdVel.publish(new ROSLIB.Message({
            linear: { x: 0, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: 0 }
        }));
    }

    function teleopLoop() {

        const gp = navigator.getGamepads()[0];

        if (!gp) {
            sendZero();
            //requestAnimationFrame(teleopLoop);
            setTimeout(teleopLoop, 25);   // ~40 Hz
            return;
        }
        teleopEnabled = gp.buttons[5]?.pressed || false;  // R1 enable
        const lx = dz(gp.axes[0] || 0);
        const ly = dz(gp.axes[1] || 0);
        const rx = dz(gp.axes[2] || 0);
        let v = 0;
        let w = 0;

        if (teleopEnabled) {
            v = -ly * 0.5;   // forward/back
            w = -rx * 0.8;   // yaw from right stick
        }

        cmdVel.publish(new ROSLIB.Message({
            linear: { x: v, y: 0, z: 0 },
            angular: { x: 0, y: 0, z: w }
        }));

        joyTopic.publish(new ROSLIB.Message({
            axes: gp.axes,
            buttons: gp.buttons.map(b => b.pressed ? 1 : 0)
        }));
        setTimeout(teleopLoop, 25);
        //requestAnimationFrame(teleopLoop);
    }
    window.addEventListener("gamepadconnected", () => {
        console.log("Gamepad connected");
    });

    teleopLoop();

    // ----- END TELEOP -----

    animate();
}