(() => {
  "use strict";

  const $ = (id) => document.getElementById(id);
  const canvas = $("simulation-canvas");
  const ctx = canvas.getContext("2d");
  const meta = {
    stack: { title: "Stack & tumble", description: "A balancing act of mass, friction, and gravity.", dimension: 3 },
    rain: { title: "Gravity rain", description: "Spheres, boxes, and a cascade of collisions.", dimension: 3 },
    pendulum: { title: "Pendulum", description: "A fixed distance. Three dimensions of freedom.", dimension: 3 },
    billiards2d: { title: "2D billiards", description: "Momentum and collisions on a two-dimensional table.", dimension: 2 },
    ccd: { title: "CCD thin-wall test", description: "The same fast shot. Continuous contact catches what a discrete step misses.", dimension: 3 },
    ccd2d: { title: "2D CCD thin-wall test", description: "Two identical circles. One thin wall. One step to see the difference.", dimension: 2 }
  };
  const palette = ["#70ddc7", "#edb576", "#86acd5", "#a69cdb", "#df9890"];
  let snapshot = null;
  let ready = false;
  let running = true;
  let scale = 1;
  let accumulator = 0;
  let previousTime = 0;
  let fps = 60;
  let statsTimestamp = 0;
  let toastTimer;
  const controls = {};
  let width = 1;
  let height = 1;
  // Physics uses a fixed tick rate, independent of the rendering frame rate.
  const tickHz = 60;
  const dt = 1 / tickHz;
  document.querySelector(".step-note").textContent = `Fixed step · ${tickHz} Hz`;
  const ccdCaption = document.querySelector("#ccd-note p");
  ccdCaption.textContent = ccdCaption.textContent.replace(/one 1\/\d+ s step/, `one 1/${tickHz} s step`);
  const isCCD = () => snapshot?.scene === "ccd" || snapshot?.scene === "ccd2d";
  const camera = { azimuth: 0.65, elevation: 0.4, distance: 19.2, target: { x: 0, y: 2.1, z: 0 } };
  let dragging = null;

  const vec = (x = 0, y = 0, z = 0) => ({ x, y, z });
  const add = (a, b) => vec(a.x + b.x, a.y + b.y, a.z + b.z);
  const sub = (a, b) => vec(a.x - b.x, a.y - b.y, a.z - b.z);
  const mul = (a, n) => vec(a.x * n, a.y * n, a.z * n);
  const dot = (a, b) => a.x * b.x + a.y * b.y + a.z * b.z;
  const cross = (a, b) => vec(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
  const normalize = (a) => mul(a, 1 / (Math.hypot(a.x, a.y, a.z) || 1));
  const clamp = (n, min, max) => Math.max(min, Math.min(max, n));
  const rotate = (v, q) => {
    if (!q) return v;
    const t = mul(cross(vec(q.x, q.y, q.z), v), 2);
    return add(v, add(mul(t, q.w), cross(vec(q.x, q.y, q.z), t)));
  };
  const position = (p) => vec(Number(p?.x) || 0, Number(p?.y) || 0, Number(p?.z) || 0);

  function color(hex, brightness = 1, alpha = 1) {
    const safe = /^#[0-9a-f]{6}$/i.test(hex || "") ? hex : "#70ddc7";
    const rgb = [1, 3, 5].map((i) => clamp(Math.round(parseInt(safe.slice(i, i + 2), 16) * brightness), 0, 255));
    return `rgba(${rgb[0]},${rgb[1]},${rgb[2]},${alpha})`;
  }

  function showToast(message) {
    $("toast").textContent = message;
    $("toast").hidden = false;
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => { $("toast").hidden = true; }, 4800);
  }

  function fatal(error) {
    ready = false;
    running = false;
    $("loading-overlay").hidden = true;
    $("fatal-overlay").hidden = false;
    $("fatal-message").textContent = error?.message || String(error);
    $("engine-status").className = "engine-status error";
    $("engine-status-text").textContent = "Engine unavailable";
    document.querySelectorAll(".engine-control").forEach((control) => { control.disabled = true; });
  }

  function bridge(method, ...args) {
    try {
      const raw = window.gosim[method](...args);
      const next = typeof raw === "string" ? JSON.parse(raw) : raw;
      if (!next || next.error) throw new Error(next?.error || "The engine returned no scene.");
      snapshot = next;
      window.demoState = snapshot;
      syncControls();
      return true;
    } catch (error) {
      showToast(error?.message || String(error));
      return false;
    }
  }

  function syncControls() {
    if (!snapshot) return;
    if (controls.scene !== snapshot.scene || controls.dimension !== snapshot.dimension) {
      controls.scene = snapshot.scene;
      controls.dimension = snapshot.dimension;
      const scene = meta[snapshot.scene] || meta.stack;
      $("scene-title").textContent = scene.title;
      $("scene-description").textContent = scene.description;
      const is2D = snapshot.dimension === 2;
      $("dimension-label").textContent = is2D ? "2D RIGID BODY SIMULATION" : "3D RIGID BODY SIMULATION";
      document.querySelectorAll("[data-scene]").forEach((button) => {
        const selected = button.dataset.scene === snapshot.scene;
        button.classList.toggle("selected", selected);
        button.setAttribute("aria-pressed", String(selected));
      });
      $("camera-hint").querySelector("span").innerHTML = is2D ? "2D view<span class=\"hint-divider\">·</span>Scroll to zoom" : "Drag to orbit<span class=\"hint-divider\">·</span>Scroll to zoom";
      $("z-label").hidden = is2D;
      $("sphere-button").lastChild.textContent = is2D ? "Circle" : "Sphere";
      $("ccd-note").hidden = !isCCD();
      // Preserve the controlled one-substep comparison. Other scenes expose all controls.
      for (const id of ["sphere-button", "box-button", "kick-button", "gravity-input", "substeps-input"]) $(id).disabled = isCCD();
      canvas.setAttribute("aria-label", is2D ? "Interactive two-dimensional physics scene. Scroll to zoom." : "Interactive three-dimensional physics scene. Drag to orbit and scroll to zoom.");
    }
    if (Number.isFinite(snapshot.gravity) && controls.gravity !== snapshot.gravity) {
      controls.gravity = snapshot.gravity;
      $("gravity-input").value = snapshot.gravity;
      $("gravity-value").innerHTML = `${snapshot.gravity.toFixed(2)} <span>m/s²</span>`;
      $("gravity-input").style.background = `linear-gradient(to right,#70ddc7 ${snapshot.gravity * 5}%,#31404e ${snapshot.gravity * 5}%)`;
    }
    if (snapshot.substeps && controls.substeps !== snapshot.substeps) {
      controls.substeps = snapshot.substeps;
      $("substeps-input").value = String(snapshot.substeps);
    }
  }

  function updateMetrics() {
    if (!snapshot) return;
    $("time-value").innerHTML = `${(Number(snapshot.time) || 0).toFixed(2)}<span>s</span>`;
    $("bodies-value").textContent = String(snapshot.bodyCount ?? snapshot.bodies?.length ?? 0);
    const energy = Number(snapshot.energy) || 0;
    $("energy-value").innerHTML = `${energy >= 10000 ? `${(energy / 1000).toFixed(1)}k` : energy.toFixed(1)}<span>J</span>`;
    const hasJoint = snapshot.joints?.length > 0;
    $("joint-value").innerHTML = `${hasJoint ? (Number(snapshot.jointError) || 0).toFixed(5) : "—"}<span>m</span>`;
  }

  function updatePlayback() {
    $("pause-label").textContent = running ? "Pause" : "Resume";
    $("pause-icon").textContent = running ? "Ⅱ" : "▶";
    $("pause-button").setAttribute("aria-label", running ? "Pause simulation" : "Resume simulation");
    $("playback-status").textContent = running ? "LIVE SIMULATION" : "SIMULATION PAUSED";
    $("playback-status").classList.toggle("paused", !running);
  }

  function setRunning(value) {
    running = Boolean(value);
    accumulator = 0;
    updatePlayback();
  }

  function resetCamera() {
    camera.azimuth = isCCD() ? 0.18 : 0.65;
    camera.elevation = isCCD() ? 0.85 : 0.4;
    camera.distance = isCCD() && snapshot.dimension === 3 ? 12 : 19.2;
    camera.target = isCCD() ? vec(1.2, 0.6, 0) : vec(0, snapshot?.scene === "pendulum" ? 2.4 : 2.1, 0);
  }

  function setScene(name) {
    if (!ready || !meta[name]) return false;
    if (!bridge("reset", name)) return false;
    accumulator = 0;
    resetCamera();
    if (isCCD()) setRunning(true);
    updateMetrics();
    return true;
  }

  function onReady() {
    if (ready || !window.gosim) return;
    ready = true;
    document.querySelectorAll(".engine-control").forEach((control) => { control.disabled = false; });
    $("loading-overlay").hidden = true;
    $("fatal-overlay").hidden = true;
    $("engine-status").className = "engine-status ready";
    $("engine-status-text").textContent = "Engine ready";
    if (!bridge("state")) {
      fatal(new Error("The physics engine could not create its initial scene."));
      return;
    }
    updatePlayback();
    updateMetrics();
    window.gosimDemo = {
      getState: () => snapshot,
      pause: () => setRunning(false),
      resume: () => setRunning(true),
      setScene,
      isRunning: () => running
    };
  }

  function resize() {
    const rect = canvas.getBoundingClientRect();
    width = Math.max(1, rect.width);
    height = Math.max(1, rect.height);
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    canvas.width = Math.round(width * dpr);
    canvas.height = Math.round(height * dpr);
    if (ctx) ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  function makeProjector() {
    if (snapshot?.dimension === 2) {
      const unit = Math.min(width / (isCCD() ? 10 : 17), height / (isCCD() ? 6 : 12)) * (19.2 / camera.distance);
      const offsetX = isCCD() ? camera.target.x : 0;
      return {
        project: (p) => ({ x: width / 2 + (p.x - offsetX) * unit, y: height / 2 - p.y * unit, depth: 20 - p.z, factor: unit }),
        eye: vec(0, 3, 100),
        is2D: true
      };
    }
    const horizontal = Math.cos(camera.elevation) * camera.distance;
    const eye = add(camera.target, vec(Math.sin(camera.azimuth) * horizontal, Math.sin(camera.elevation) * camera.distance, Math.cos(camera.azimuth) * horizontal));
    const forward = normalize(sub(camera.target, eye));
    const right = normalize(cross(forward, vec(0, 1, 0)));
    const up = cross(right, forward);
    const focal = Math.min(width * 0.95, height * 1.4);
    return {
      project: (p) => {
        const offset = sub(p, eye);
        const depth = dot(offset, forward);
        if (depth < 0.2) return null;
        const factor = focal / depth;
        return { x: width / 2 + dot(offset, right) * factor, y: height / 2 - dot(offset, up) * factor, depth, factor };
      },
      eye,
      is2D: false
    };
  }

  function path(points, close = true) {
    if (!points.length || points.some((p) => !p)) return false;
    ctx.beginPath();
    ctx.moveTo(points[0].x, points[0].y);
    for (let i = 1; i < points.length; i++) ctx.lineTo(points[i].x, points[i].y);
    if (close) ctx.closePath();
    return true;
  }

  function line3D(projector, a, b, stroke, lineWidth = 1) {
    if (!path([projector.project(a), projector.project(b)], false)) return;
    ctx.strokeStyle = stroke;
    ctx.lineWidth = lineWidth;
    ctx.stroke();
  }

  function drawGrid(projector) {
    if (projector.is2D) {
      for (let x = -12; x <= 12; x++) line3D(projector, vec(x, -3, 0), vec(x, 13, 0), x === 0 ? "#42677666" : "#45617322");
      for (let y = -3; y <= 13; y++) line3D(projector, vec(-12, y, 0), vec(12, y, 0), y === 0 ? "#42677666" : "#45617322");
      return;
    }
    const floor = [vec(-12, 0, -12), vec(12, 0, -12), vec(12, 0, 12), vec(-12, 0, 12)].map(projector.project);
    if (path(floor)) {
      ctx.fillStyle = "#10232b70";
      ctx.fill();
    }
    for (let i = -12; i <= 12; i++) {
      const stroke = i % 5 === 0 ? "#658d9c32" : "#587d8f1c";
      line3D(projector, vec(i, 0.004, -12), vec(i, 0.004, 12), stroke);
      line3D(projector, vec(-12, 0.004, i), vec(12, 0.004, i), stroke);
    }
    line3D(projector, vec(-12, 0.006, 0), vec(12, 0.006, 0), "#b7806a40");
    line3D(projector, vec(0, 0.006, -12), vec(0, 0.006, 12), "#759ab840");
    for (const [point, text, shade] of [[vec(7, 0.02, 0), "X", "#d4967b"], [vec(0, 0.02, 7), "Z", "#87adca"]]) {
      const p = projector.project(point);
      if (!p) continue;
      ctx.font = "9px 'Segoe UI', sans-serif";
      ctx.fillStyle = shade;
      ctx.fillText(text, p.x + 6, p.y - 5);
    }
  }

  function drawShadows(projector, bodies) {
    if (projector.is2D) return;
    for (const body of bodies) {
      if (body.static) continue;
      const p = position(body.position);
      const ext = body.halfExtents;
      const radius = body.radius || Math.max(ext?.x || 0.4, ext?.z || 0.4);
      const spread = radius * (1 + Math.max(0, p.y) * 0.05);
      const points = [];
      for (let i = 0; i < 24; i++) {
        const angle = i * Math.PI / 12;
        points.push(projector.project(vec(p.x + Math.cos(angle) * spread, 0.012, p.z + Math.sin(angle) * spread)));
      }
      if (path(points)) {
        ctx.fillStyle = `rgba(3,10,17,${clamp(0.26 - p.y * 0.018, 0.04, 0.24)})`;
        ctx.fill();
      }
    }
  }

  const cubeFaces = [
    { indices: [0, 3, 2, 1], normal: vec(0, 0, -1) },
    { indices: [4, 5, 6, 7], normal: vec(0, 0, 1) },
    { indices: [0, 4, 7, 3], normal: vec(-1, 0, 0) },
    { indices: [1, 2, 6, 5], normal: vec(1, 0, 0) },
    { indices: [0, 1, 5, 4], normal: vec(0, -1, 0) },
    { indices: [3, 7, 6, 2], normal: vec(0, 1, 0) }
  ];
  const light = normalize(vec(-0.45, 0.85, 0.7));

  function bodyPrimitives(projector, body, index) {
    const center = position(body.position);
    const shade = body.color || (body.static ? "#456379" : palette[index % palette.length]);
    if (body.shape === "sphere" || body.shape === "circle") {
      const p = projector.project(center);
      if (!p) return [];
      return [{ kind: "sphere", p, radius: Math.max(1, (body.radius || 0.5) * p.factor), color: shade, depth: p.depth, static: body.static }];
    }
    const ext = position(body.halfExtents);
    if (!projector.is2D && body.static && ext.x >= 4 && ext.z >= 4 && ext.y < 0.65 && center.y <= 0) return [];
    if (projector.is2D) {
      const local = [vec(-ext.x, -ext.y, 0), vec(ext.x, -ext.y, 0), vec(ext.x, ext.y, 0), vec(-ext.x, ext.y, 0)];
      const points = local.map((v) => projector.project(add(center, rotate(v, body.rotation))));
      if (points.some((p) => !p)) return [];
      return [{ kind: "face", points, color: shade, brightness: body.static ? 0.65 : 0.9, depth: 20 - center.z, static: body.static }];
    }
    const local = [
      vec(-ext.x, -ext.y, -ext.z), vec(ext.x, -ext.y, -ext.z), vec(ext.x, ext.y, -ext.z), vec(-ext.x, ext.y, -ext.z),
      vec(-ext.x, -ext.y, ext.z), vec(ext.x, -ext.y, ext.z), vec(ext.x, ext.y, ext.z), vec(-ext.x, ext.y, ext.z)
    ];
    const world = local.map((v) => add(center, rotate(v, body.rotation)));
    const points = world.map(projector.project);
    const faces = [];
    for (const face of cubeFaces) {
      const normal = rotate(face.normal, body.rotation);
      const faceCenter = mul(face.indices.reduce((sum, i) => add(sum, world[i]), vec()), 0.25);
      if (dot(normal, sub(projector.eye, faceCenter)) <= 0) continue;
      const polygon = face.indices.map((i) => points[i]);
      if (polygon.some((p) => !p)) continue;
      faces.push({ kind: "face", points: polygon, color: shade, brightness: (body.static ? 0.4 : 0.56) + Math.max(0, dot(normal, light)) * 0.46, depth: polygon.reduce((n, p) => n + p.depth, 0) / 4, static: body.static });
    }
    return faces;
  }

  function drawPrimitive(primitive) {
    if (primitive.kind === "face") {
      if (!path(primitive.points)) return;
      ctx.fillStyle = color(primitive.color, primitive.brightness);
      ctx.fill();
      ctx.strokeStyle = primitive.static ? "#8ba8b325" : color(primitive.color, 1.13, 0.6);
      ctx.lineWidth = 0.8;
      ctx.lineJoin = "round";
      ctx.stroke();
    } else if (primitive.kind === "sphere") {
      const { p, radius, static: fixed } = primitive;
      if (radius > 100000) return;
      const gradient = ctx.createRadialGradient(p.x - radius * 0.35, p.y - radius * 0.4, radius * 0.03, p.x + radius * 0.12, p.y + radius * 0.1, radius * 1.05);
      gradient.addColorStop(0, color(primitive.color, fixed ? 1 : 1.2));
      gradient.addColorStop(0.5, color(primitive.color, fixed ? 0.73 : 0.9));
      gradient.addColorStop(1, color(primitive.color, 0.35));
      ctx.beginPath();
      ctx.arc(p.x, p.y, radius, 0, Math.PI * 2);
      ctx.fillStyle = gradient;
      ctx.fill();
      ctx.strokeStyle = color(primitive.color, 1.1, 0.6);
      ctx.lineWidth = 0.7;
      ctx.stroke();
      ctx.beginPath();
      ctx.ellipse(p.x - radius * 0.32, p.y - radius * 0.39, radius * 0.25, radius * 0.12, -0.5, 0, Math.PI * 2);
      ctx.fillStyle = "#e9fffa18";
      ctx.fill();
    } else if (primitive.kind === "joint") {
      if (!path([primitive.a, primitive.b], false)) return;
      ctx.strokeStyle = "#efba75";
      ctx.lineWidth = 2.3;
      ctx.lineCap = "round";
      ctx.stroke();
      for (const p of [primitive.a, primitive.b]) {
        ctx.beginPath();
        ctx.arc(p.x, p.y, 3.5, 0, Math.PI * 2);
        ctx.fillStyle = "#f5ce98";
        ctx.fill();
      }
    }
  }

  function render() {
    if (!ctx) return;
    ctx.clearRect(0, 0, width, height);
    const projector = makeProjector();
    drawGrid(projector);
    const bodies = snapshot?.bodies || [];
    drawShadows(projector, bodies);
    const primitives = [];
    bodies.forEach((body, index) => { primitives.push(...bodyPrimitives(projector, body, index)); });
    for (const joint of snapshot?.joints || []) {
      const a = projector.project(position(joint.a));
      const b = projector.project(position(joint.b));
      if (a && b) primitives.push({ kind: "joint", a, b, depth: (a.depth + b.depth) / 2 });
    }
    primitives.sort((a, b) => b.depth - a.depth);
    for (const primitive of primitives) drawPrimitive(primitive);
    if (isCCD()) drawCCDAnnotations(projector, bodies);
  }

  function drawCCDAnnotations(projector, bodies) {
    ctx.save();
    for (const body of bodies) {
      if (body.static) continue;
      const current = position(body.position);
      const start = projector.project(vec(-2, current.y, current.z));
      const end = projector.project(current);
      if (!start || !end) continue;
      // These are measurement guides from the scene's initial position to the
      // actual Go snapshot, not a second simulation or an animated trajectory.
      ctx.setLineDash([4, 5]);
      line3D(projector, vec(-2, current.y, current.z), current, color(body.color, 1, 0.55), 1.5);
      ctx.setLineDash([]);
      ctx.strokeStyle = color(body.color, 1, 0.8);
      ctx.lineWidth = 1.3;
      for (const [point, radius] of [[start, 4], [end, Math.max(7, body.radius * end.factor + 4)]]) {
        ctx.beginPath();
        ctx.arc(point.x, point.y, radius, 0, Math.PI * 2);
        ctx.stroke();
      }
      const labelX = clamp(end.x + 13, 12, Math.max(12, width - 145));
      const labelY = clamp(end.y - 15, 80, height - 60);
      ctx.font = "600 11px 'Segoe UI', sans-serif";
      ctx.fillStyle = body.color;
      ctx.fillText(body.continuous ? "CCD ON" : "CCD OFF · discrete", labelX, labelY);
      ctx.font = "11px ui-monospace, monospace";
      ctx.fillStyle = "#d4e1e9";
      ctx.fillText(`x = ${current.x.toFixed(3)} m`, labelX, labelY + 17);
    }
    const wall = projector.project(projector.is2D ? vec(0, 2.1) : vec(0, 2.4, 0));
    if (wall) {
      ctx.font = "10px 'Segoe UI', sans-serif";
      ctx.fillStyle = "#a8bac9";
      ctx.fillText("0.02 m wall", wall.x - 29, wall.y - 12);
    }
    ctx.restore();
  }

  function frame(timestamp) {
    const elapsed = previousTime ? clamp((timestamp - previousTime) / 1000, 0, 0.25) : 0;
    previousTime = timestamp;
    if (elapsed > 0) fps += (1 / elapsed - fps) * 0.045;
    if (ready && running) {
      accumulator += elapsed * scale;
      let steps = 0;
      while (accumulator >= dt && steps < 8) {
        if (!bridge("step", dt)) {
          setRunning(false);
          break;
        }
        accumulator -= dt;
        steps++;
        if (isCCD()) {
          setRunning(false);
          updateMetrics();
          break;
        }
      }
      // Bound catch-up work and explicitly discard delayed time after a stall.
      if (steps === 8 && accumulator >= dt) accumulator %= dt;
    }
    render();
    if (timestamp - statsTimestamp > 120) {
      updateMetrics();
      $("fps-value").textContent = String(Math.round(clamp(fps, 0, 240)));
      statsTimestamp = timestamp;
    }
    requestAnimationFrame(frame);
  }

  document.querySelectorAll("[data-scene]").forEach((button) => button.addEventListener("click", () => setScene(button.dataset.scene)));
  $("pause-button").addEventListener("click", () => setRunning(!running));
  $("step-button").addEventListener("click", () => {
    setRunning(false);
    if (ready) bridge("step", dt);
    updateMetrics();
  });
  $("reset-button").addEventListener("click", () => setScene(snapshot?.scene || "stack"));
  $("sphere-button").addEventListener("click", () => { if (ready) bridge("spawn", "sphere"); });
  $("box-button").addEventListener("click", () => { if (ready) bridge("spawn", "box"); });
  $("kick-button").addEventListener("click", () => { if (ready) bridge("kick"); });
  $("gravity-input").addEventListener("input", (event) => { if (ready) bridge("configure", JSON.stringify({ gravity: Number(event.target.value) })); });
  $("speed-input").addEventListener("change", (event) => { scale = Number(event.target.value); accumulator = 0; });
  $("substeps-input").addEventListener("change", (event) => { if (ready) bridge("configure", JSON.stringify({ substeps: Number(event.target.value) })); });
  $("view-button").addEventListener("click", resetCamera);
  $("reload-button").addEventListener("click", () => window.location.reload());
  document.querySelector(".brand").addEventListener("click", (event) => event.preventDefault());

  document.addEventListener("keydown", (event) => {
    if (!ready || /INPUT|SELECT|TEXTAREA|BUTTON/.test(event.target.tagName) || event.target.isContentEditable) return;
    if (event.code === "Space") {
      event.preventDefault();
      setRunning(!running);
    } else if (event.key.toLowerCase() === "r") {
      setScene(snapshot?.scene || "stack");
    }
  });
  canvas.addEventListener("pointerdown", (event) => {
    if (event.button !== 0 || snapshot?.dimension === 2) return;
    dragging = { x: event.clientX, y: event.clientY, id: event.pointerId };
    canvas.setPointerCapture(event.pointerId);
    canvas.classList.add("dragging");
  });
  canvas.addEventListener("pointermove", (event) => {
    if (!dragging || event.pointerId !== dragging.id) return;
    camera.azimuth -= (event.clientX - dragging.x) * 0.008;
    camera.elevation = clamp(camera.elevation + (event.clientY - dragging.y) * 0.006, 0.12, 1.3);
    dragging.x = event.clientX;
    dragging.y = event.clientY;
  });
  const endDrag = () => { dragging = null; canvas.classList.remove("dragging"); };
  canvas.addEventListener("pointerup", endDrag);
  canvas.addEventListener("pointercancel", endDrag);
  canvas.addEventListener("lostpointercapture", endDrag);
  canvas.addEventListener("wheel", (event) => {
    event.preventDefault();
    camera.distance = clamp(camera.distance * Math.exp(event.deltaY * 0.001), 8, 45);
  }, { passive: false });
  document.addEventListener("visibilitychange", () => { accumulator = 0; previousTime = 0; });
  window.addEventListener("gosim-ready", onReady);
  window.addEventListener("resize", resize);
  if (window.ResizeObserver) new ResizeObserver(resize).observe($("stage"));
  resize();
  updatePlayback();
  requestAnimationFrame(frame);

  async function boot() {
    try {
      if (!ctx) throw new Error("This browser does not support Canvas 2D rendering.");
      if (window.gosim) { onReady(); return; }
      if (!window.WebAssembly || !window.Go) throw new Error("Open this file in a modern browser with WebAssembly support.");
      const data = $("gosim-wasm");
      if (!data?.textContent.trim()) throw new Error("This is the source template. Open the generated demo.html file, or run the browser example from the repository.");
      const bytes = Uint8Array.from(atob(data.textContent.trim()), (character) => character.charCodeAt(0));
      const go = new Go();
      const result = await WebAssembly.instantiate(bytes, go.importObject);
      go.run(result.instance).catch(fatal);
      if (window.gosim) onReady();
    } catch (error) {
      fatal(error);
    }
  }
  boot();
})();
