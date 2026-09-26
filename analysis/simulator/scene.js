/*
 * Dependency-free WebGL tram telemetry scene.
 *
 * Include with <script src="scene.js"></script>, then:
 *   const scene = window.TramScene.create(document.querySelector('#scene-canvas'));
 *   scene.setState({t, velocity, front, rear, command, distance,
 *                   slipFront, slipRear, heading});
 *   scene.resize();
 *   scene.dispose();
 *
 * t is bag-relative seconds; speeds are metres/second, distance metres.  The
 * Pass null for a missing front/rear reading; its wheel freezes and bogie
 * becomes grey.  The vehicle is a schematic, not a dimensioned CAD model.  A stationary camera
 * follows it while sleepers scroll and the front/rear wheels spin at their
 * respective measured speeds.  No external resources are loaded.
 */
(function () {
  'use strict';

  const TAU = Math.PI * 2;
  const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
  const finite = (x, fallback = 0) =>
    x == null || x === '' ? fallback : (Number.isFinite(+x) ? +x : fallback);

  function mat4() {
    return new Float32Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
  }
  function mul(a, b) {
    const out = new Float32Array(16);
    for (let c = 0; c < 4; c++) {
      for (let r = 0; r < 4; r++) {
        out[c * 4 + r] = a[r] * b[c * 4] + a[4 + r] * b[c * 4 + 1] +
          a[8 + r] * b[c * 4 + 2] + a[12 + r] * b[c * 4 + 3];
      }
    }
    return out;
  }
  function translate(x, y, z) {
    const m = mat4(); m[12] = x; m[13] = y; m[14] = z; return m;
  }
  function scale(x, y, z) {
    const m = mat4(); m[0] = x; m[5] = y; m[10] = z; return m;
  }
  function rotateY(t) {
    const m = mat4(), c = Math.cos(t), s = Math.sin(t);
    m[0] = c; m[2] = -s; m[8] = s; m[10] = c; return m;
  }
  function rotateZ(t) {
    const m = mat4(), c = Math.cos(t), s = Math.sin(t);
    m[0] = c; m[1] = s; m[4] = -s; m[5] = c; return m;
  }
  function perspective(fovy, aspect, near, far) {
    const f = 1 / Math.tan(fovy / 2), out = new Float32Array(16);
    out[0] = f / aspect; out[5] = f; out[10] = (far + near) / (near - far);
    out[11] = -1; out[14] = 2 * far * near / (near - far); return out;
  }
  function lookAt(eye, target) {
    let f = [target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]];
    let n = Math.hypot(...f); f = f.map(v => v / n);
    let s = [-f[2], 0, f[0]]; n = Math.hypot(...s); s = s.map(v => v / n);
    const u = [s[1] * f[2] - s[2] * f[1], s[2] * f[0] - s[0] * f[2], s[0] * f[1] - s[1] * f[0]];
    return new Float32Array([
      s[0], u[0], -f[0], 0,
      s[1], u[1], -f[1], 0,
      s[2], u[2], -f[2], 0,
      -s[0] * eye[0] - s[1] * eye[1] - s[2] * eye[2],
      -u[0] * eye[0] - u[1] * eye[1] - u[2] * eye[2],
      f[0] * eye[0] + f[1] * eye[1] + f[2] * eye[2], 1
    ]);
  }
  function boxMesh() {
    const vertices = [];
    const faces = [
      [[1, 0, 0], [[.5, -.5, -.5], [.5, .5, -.5], [.5, .5, .5], [.5, -.5, .5]]],
      [[-1, 0, 0], [[-.5, -.5, .5], [-.5, .5, .5], [-.5, .5, -.5], [-.5, -.5, -.5]]],
      [[0, 1, 0], [[-.5, .5, -.5], [-.5, .5, .5], [.5, .5, .5], [.5, .5, -.5]]],
      [[0, -1, 0], [[-.5, -.5, .5], [-.5, -.5, -.5], [.5, -.5, -.5], [.5, -.5, .5]]],
      [[0, 0, 1], [[-.5, -.5, .5], [.5, -.5, .5], [.5, .5, .5], [-.5, .5, .5]]],
      [[0, 0, -1], [[.5, -.5, -.5], [-.5, -.5, -.5], [-.5, .5, -.5], [.5, .5, -.5]]]
    ];
    for (const [normal, p] of faces) {
      for (const i of [0, 1, 2, 0, 2, 3]) vertices.push(...p[i], ...normal);
    }
    return new Float32Array(vertices);
  }
  function cylinderMesh(segments = 28) {
    const out = [];
    function v(x, y, z, nx, ny, nz) { out.push(x, y, z, nx, ny, nz); }
    for (let i = 0; i < segments; i++) {
      const a = i * TAU / segments, b = (i + 1) * TAU / segments;
      const ca = Math.cos(a), sa = Math.sin(a), cb = Math.cos(b), sb = Math.sin(b);
      v(ca, sa, -.5, ca, sa, 0); v(cb, sb, -.5, cb, sb, 0); v(cb, sb, .5, cb, sb, 0);
      v(ca, sa, -.5, ca, sa, 0); v(cb, sb, .5, cb, sb, 0); v(ca, sa, .5, ca, sa, 0);
      v(0, 0, .5, 0, 0, 1); v(ca, sa, .5, 0, 0, 1); v(cb, sb, .5, 0, 0, 1);
      v(0, 0, -.5, 0, 0, -1); v(cb, sb, -.5, 0, 0, -1); v(ca, sa, -.5, 0, 0, -1);
    }
    return new Float32Array(out);
  }
  function makeShader(gl, type, source) {
    const shader = gl.createShader(type);
    gl.shaderSource(shader, source); gl.compileShader(shader);
    if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
      const error = gl.getShaderInfoLog(shader); gl.deleteShader(shader); throw Error(error);
    }
    return shader;
  }
  function makeProgram(gl) {
    const vs = makeShader(gl, gl.VERTEX_SHADER, `
      attribute vec3 aPosition;
      attribute vec3 aNormal;
      uniform mat4 uViewProjection;
      uniform mat4 uModel;
      varying vec3 vNormal;
      varying float vDepth;
      void main() {
        vec4 world = uModel * vec4(aPosition, 1.0);
        gl_Position = uViewProjection * world;
        vNormal = normalize(mat3(uModel) * aNormal);
        vDepth = length(world.xyz);
      }
    `);
    const fs = makeShader(gl, gl.FRAGMENT_SHADER, `
      precision mediump float;
      uniform vec3 uColor;
      varying vec3 vNormal;
      varying float vDepth;
      void main() {
        vec3 light = normalize(vec3(0.4, 0.8, 0.55));
        float diffuse = max(dot(normalize(vNormal), light), 0.0);
        float illumination = 0.51 + 0.49 * diffuse;
        vec3 color = uColor * illumination;
        float haze = smoothstep(17.0, 42.0, vDepth) * 0.22;
        gl_FragColor = vec4(mix(color, vec3(0.06, 0.10, 0.16), haze), 1.0);
      }
    `);
    const program = gl.createProgram();
    gl.attachShader(program, vs); gl.attachShader(program, fs); gl.linkProgram(program);
    gl.deleteShader(vs); gl.deleteShader(fs);
    if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
      const error = gl.getProgramInfoLog(program); gl.deleteProgram(program); throw Error(error);
    }
    return program;
  }

  const COLOR = {
    ground: [0.105, 0.159, 0.216], ballast: [0.235, 0.280, 0.302],
    sleeper: [0.19, 0.22, 0.25], rail: [0.69, 0.75, 0.78], railFace: [0.42, 0.48, 0.52],
    body: [0.15, 0.30, 0.53], bodyRoof: [0.17, 0.22, 0.30],
    lower: [0.20, 0.24, 0.30], stripe: [0.90, 0.15, 0.16],
    window: [0.12, 0.59, 0.69], windowDark: [0.05, 0.27, 0.38],
    trim: [0.82, 0.87, 0.88], wheel: [0.16, 0.19, 0.22],
    hub: [0.54, 0.61, 0.65], spoke: [0.88, 0.94, 0.97],
    slip: [1.0, 0.19, 0.11], front: [0.12, 0.80, 0.89], rear: [1.0, 0.59, 0.19],
    missing: [0.34, 0.40, 0.45],
    light: [1.0, 0.86, 0.48]
  };

  function create(canvas) {
    if (!(canvas instanceof HTMLCanvasElement)) throw Error('TramScene.create expects a canvas element');
    const gl = canvas.getContext('webgl', {antialias: true, alpha: false, powerPreference: 'low-power'});
    if (!gl) {
      const ctx = canvas.getContext('2d');
      if (ctx) {
        ctx.fillStyle = '#0b1826'; ctx.fillRect(0, 0, canvas.width, canvas.height);
        ctx.fillStyle = '#b8d6e2'; ctx.font = '16px sans-serif';
        ctx.fillText('Для 3D-визуализации нужен WebGL', 18, 32);
      }
      return {setState() {}, resize() {}, dispose() {}, available: false};
    }
    const program = makeProgram(gl);
    const aPosition = gl.getAttribLocation(program, 'aPosition');
    const aNormal = gl.getAttribLocation(program, 'aNormal');
    const uViewProjection = gl.getUniformLocation(program, 'uViewProjection');
    const uModel = gl.getUniformLocation(program, 'uModel');
    const uColor = gl.getUniformLocation(program, 'uColor');
    function mesh(data) {
      const buffer = gl.createBuffer(); gl.bindBuffer(gl.ARRAY_BUFFER, buffer);
      gl.bufferData(gl.ARRAY_BUFFER, data, gl.STATIC_DRAW);
      return {buffer, count: data.length / 6};
    }
    const cube = mesh(boxMesh()), cylinder = mesh(cylinderMesh());
    const meshes = [cube, cylinder];
    let width = 1, height = 1, yaw = .52, pitch = .31, radius = 13.0;
    let pointer = null, frame = 0, disposed = false, dirty = true;
    let state = {t: null, velocity: 0, front: 0, rear: 0, command: 0,
      distance: 0, slipFront: false, slipRear: false, heading: 0};
    let frontAngle = 0, rearAngle = 0;

    function resize() {
      const dpr = clamp(window.devicePixelRatio || 1, 1, 2);
      const rect = canvas.getBoundingClientRect();
      const w = Math.max(1, Math.round((rect.width || canvas.clientWidth || 640) * dpr));
      const h = Math.max(1, Math.round((rect.height || canvas.clientHeight || 420) * dpr));
      if (canvas.width !== w || canvas.height !== h) {
        canvas.width = w; canvas.height = h; width = w; height = h;
        gl.viewport(0, 0, w, h); dirty = true;
      }
    }
    function draw(meshObject, model, color) {
      gl.bindBuffer(gl.ARRAY_BUFFER, meshObject.buffer);
      gl.vertexAttribPointer(aPosition, 3, gl.FLOAT, false, 24, 0);
      gl.vertexAttribPointer(aNormal, 3, gl.FLOAT, false, 24, 12);
      gl.uniformMatrix4fv(uModel, false, model);
      gl.uniform3fv(uColor, color);
      gl.drawArrays(gl.TRIANGLES, 0, meshObject.count);
    }
    function box(x, y, z, sx, sy, sz, color, rz = 0, ry = 0) {
      let m = translate(x, y, z);
      if (ry) m = mul(m, rotateY(ry));
      if (rz) m = mul(m, rotateZ(rz));
      draw(cube, mul(m, scale(sx, sy, sz)), color);
    }
    function wheel(x, z, angle, slip, side, accent) {
      const y = .49, color = slip ? COLOR.slip : COLOR.wheel;
      const base = mul(translate(x, y, z), rotateZ(angle));
      draw(cylinder, mul(base, scale(.355, .355, .18)), color);
      const outside = z + side * .105;
      draw(cylinder, mul(mul(translate(x, y, outside), rotateZ(angle)), scale(.16, .16, .022)), COLOR.hub);
      for (const spoke of [0, Math.PI / 2]) {
        const spokeTransform = mul(mul(translate(x, y, outside + side * .02), rotateZ(angle + spoke)), scale(.055, .50, .028));
        draw(cube, spokeTransform, slip ? COLOR.slip : accent);
      }
    }
    function render() {
      if (disposed) return;
      resize();
      if (!dirty || document.hidden) return;
      dirty = false;
      gl.clearColor(.045, .075, .12, 1);
      gl.clear(gl.COLOR_BUFFER_BIT | gl.DEPTH_BUFFER_BIT);
      gl.enable(gl.DEPTH_TEST);
      gl.useProgram(program);
      gl.enableVertexAttribArray(aPosition); gl.enableVertexAttribArray(aNormal);
      const eye = [radius * Math.sin(yaw) * Math.cos(pitch), 1.35 + radius * Math.sin(pitch),
        radius * Math.cos(yaw) * Math.cos(pitch)];
      const vp = mul(perspective(Math.PI / 4, width / height, .1, 90), lookAt(eye, [0, 1.35, 0]));
      gl.uniformMatrix4fv(uViewProjection, false, vp);

      // Rails and sleepers scroll under the fixed vehicle.  GNSS trajectory is
      // handled by the map view; this is a local schematic and has no turn model.
      box(0, -.14, 0, 64, .12, 8, COLOR.ground);
      box(0, -.04, 0, 64, .11, 2.6, COLOR.ballast);
      const sleeperSpacing = 1.1;
      const offset = ((finite(state.distance) % sleeperSpacing) + sleeperSpacing) % sleeperSpacing;
      for (let i = -22; i <= 22; i++) {
        box(i * sleeperSpacing - offset, .045, 0, .22, .13, 2.35, COLOR.sleeper);
      }
      for (const z of [-.72, .72]) {
        box(0, .14, z, 64, .14, .11, COLOR.railFace);
        box(0, .23, z, 64, .045, .15, COLOR.rail);
      }

      // Two bogies, each with two axles and four visible wheels.
      for (const [center, angle, slip, accent] of [
        [2.45, frontAngle, !!state.slipFront, state.front == null ? COLOR.missing : COLOR.front],
        [-2.45, rearAngle, !!state.slipRear, state.rear == null ? COLOR.missing : COLOR.rear]
      ]) {
        box(center, .73, 0, 2.02, .25, 1.66, COLOR.lower);
        box(center, .96, 0, 1.42, .22, 1.35, slip ? COLOR.slip : accent);
        for (const dx of [-.54, .54]) {
          box(center + dx, .49, 0, .09, .09, 1.78, COLOR.hub);
          for (const side of [-1, 1]) wheel(center + dx, side * .82, angle, slip, side, accent);
        }
      }

      // Short articulated-looking tram silhouette; front is positive X.
      box(0, 1.63, 0, 8.7, 1.46, 2.20, COLOR.body);
      box(0, 2.48, 0, 8.55, .30, 2.13, COLOR.bodyRoof);
      box(0, 1.07, 0, 8.72, .29, 2.23, COLOR.lower);
      box(0, 1.53, 1.118, 8.58, .135, .035, COLOR.stripe);
      box(0, 1.53, -1.118, 8.58, .135, .035, COLOR.stripe);
      for (const side of [-1, 1]) {
        for (const x of [-3.35, -2.28, -1.21, -.14, .93, 2.0, 3.07]) {
          box(x, 2.04, side * 1.118, .79, .52, .025, COLOR.window);
          box(x + .37, 2.04, side * 1.133, .028, .52, .02, COLOR.trim);
        }
        // Door seam and handles visibly locate vehicle ends.
        box(-.67, 1.67, side * 1.143, .025, 1.22, .02, COLOR.trim);
        box(.0, 1.67, side * 1.143, .025, 1.22, .02, COLOR.trim);
        box(-.34, 1.45, side * 1.158, .11, .025, .02, COLOR.trim);
      }
      box(4.385, 2.03, 0, .04, .74, 1.64, COLOR.windowDark);
      box(-4.385, 2.03, 0, .04, .74, 1.64, COLOR.windowDark);
      for (const z of [-.69, .69]) {
        box(4.415, 1.26, z, .07, .17, .23, COLOR.light);
        box(-4.415, 1.26, z, .07, .17, .23, COLOR.stripe);
      }
      // Two rooftop modules indicate that this is a powered railcar.
      for (const x of [-2.1, 2.0]) box(x, 2.72, 0, 1.15, .19, 1.14, COLOR.lower);
      // Diamond pantograph: a visual identity cue, with intentionally simple
      // geometry because the underlying bags contain no pantograph telemetry.
      box(.3, 2.75, 0, 1.05, .07, .62, COLOR.trim);
      box(.05, 2.99, 0, .62, .045, .065, COLOR.trim, .61);
      box(.55, 2.99, 0, .62, .045, .065, COLOR.trim, -.61);
      box(.05, 3.34, 0, .62, .045, .065, COLOR.trim, -.61);
      box(.55, 3.34, 0, .62, .045, .065, COLOR.trim, .61);
      box(.3, 3.55, 0, 1.21, .04, .07, COLOR.trim);
    }
    function tick() {
      if (disposed) return;
      render(); frame = requestAnimationFrame(tick);
    }
    function setState(next) {
      if (!next || typeof next !== 'object') return;
      const previousT = state.t;
      const t = next.t == null ? previousT : finite(next.t, previousT);
      const dt = previousT == null || t == null ? 0 : clamp(t - previousT, -15, 15);
      const front = Object.prototype.hasOwnProperty.call(next, 'front') && next.front == null
        ? null : finite(next.front, state.front);
      const rear = Object.prototype.hasOwnProperty.call(next, 'rear') && next.rear == null
        ? null : finite(next.rear, state.rear);
      if (front != null) frontAngle = (frontAngle - front * dt / .355) % TAU;
      if (rear != null) rearAngle = (rearAngle - rear * dt / .355) % TAU;
      state = {
        t, velocity: finite(next.velocity, state.velocity), front, rear,
        command: finite(next.command, state.command),
        distance: finite(next.distance, state.distance),
        slipFront: next.slipFront == null ? state.slipFront : !!next.slipFront,
        slipRear: next.slipRear == null ? state.slipRear : !!next.slipRear,
        heading: finite(next.heading, state.heading)
      };
      dirty = true;
    }
    function onDown(event) {
      pointer = {id: event.pointerId, x: event.clientX, y: event.clientY};
      canvas.setPointerCapture(event.pointerId);
    }
    function onMove(event) {
      if (!pointer || pointer.id !== event.pointerId) return;
      yaw += (event.clientX - pointer.x) * .008;
      pitch = clamp(pitch + (event.clientY - pointer.y) * .008, .08, 1.33);
      pointer.x = event.clientX; pointer.y = event.clientY; dirty = true;
    }
    function onUp(event) {
      if (pointer && pointer.id === event.pointerId) pointer = null;
    }
    function onWheel(event) {
      event.preventDefault();
      radius = clamp(radius * Math.exp(event.deltaY * .001), 6.5, 33);
      dirty = true;
    }
    canvas.style.touchAction = 'none';
    canvas.addEventListener('pointerdown', onDown);
    canvas.addEventListener('pointermove', onMove);
    canvas.addEventListener('pointerup', onUp);
    canvas.addEventListener('pointercancel', onUp);
    canvas.addEventListener('wheel', onWheel, {passive: false});
    const observer = typeof ResizeObserver !== 'undefined' ? new ResizeObserver(resize) : null;
    if (observer) observer.observe(canvas);
    resize(); tick();
    function dispose() {
      if (disposed) return;
      disposed = true; cancelAnimationFrame(frame);
      if (observer) observer.disconnect();
      canvas.removeEventListener('pointerdown', onDown);
      canvas.removeEventListener('pointermove', onMove);
      canvas.removeEventListener('pointerup', onUp);
      canvas.removeEventListener('pointercancel', onUp);
      canvas.removeEventListener('wheel', onWheel);
      for (const m of meshes) gl.deleteBuffer(m.buffer);
      gl.deleteProgram(program);
    }
    return {setState, resize, dispose, available: true};
  }

  window.TramScene = Object.freeze({create});
})();
