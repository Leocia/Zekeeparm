/* Reuse the project's Three.js / URDF loaders and original CAD meshes. */
(() => {
  const host = document.getElementById('scene');
  const status = document.getElementById('model-status');
  try {
    let needsRender = true;
    const scene = new THREE.Scene();
    scene.background = new THREE.Color('#19231f');
    const camera = new THREE.PerspectiveCamera(42, 1, .01, 10);
    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
    renderer.outputEncoding = THREE.sRGBEncoding;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = .9;
    host.appendChild(renderer.domElement);
    scene.add(new THREE.HemisphereLight(0xffffff, 0x31443a, .9));
    const light = new THREE.DirectionalLight(0xffffff, 1.0);
    light.position.set(1, 2, 1); scene.add(light);
    const grid = new THREE.GridHelper(1.8, 18, 0x5c7062, 0x33463a);
    grid.position.y = -.001; scene.add(grid);
    const frame = new THREE.Group(); frame.rotation.x = -Math.PI / 2; scene.add(frame);
    frame.add(new THREE.AxesHelper(.12));
    let robot, values = [0, .7, .8, -.75, 0, 0], opening = 40, collisionLinks = new Set();
    let theta = 1.1, phi = 1.15, radius = 1.05, pointer;
    function cameraPosition() {
      camera.position.set(.15 + radius*Math.sin(phi)*Math.cos(theta), .2 + radius*Math.cos(phi), radius*Math.sin(phi)*Math.sin(theta));
      camera.lookAt(.15, .2, 0);
      needsRender = true;
    }
    cameraPosition();
    const manager = new THREE.LoadingManager();
    manager.onLoad = () => {
      if(!robot)return;
      robot.traverse(child => {
        if(child.isMesh) child.material = new THREE.MeshStandardMaterial({color:'#d8c9ad',metalness:.38,roughness:.52});
      });
      window.StudioScene.setCollisionLinks([...collisionLinks]);
      status.hidden = true;
      needsRender = true;
    };
    const loader = new URDFLoader(manager);
    loader.packages = { zekeep_bringup: '/model' };
    loader.load('/model/description/urdf/sixaxis.urdf', loaded => {
      robot = loaded; frame.add(robot); apply(values,opening);
    }, undefined, error => { status.textContent = `模型加载失败：${error.message || error}`; });
    function apply(joints,width) {
      if (!robot) return;
      joints.forEach((value,i) => robot.joints[`joint${i+1}`]?.setJointValue(value));
      robot.joints.gripper_joint?.setJointValue(width / 2000);
      robot.joints.right_joint?.setJointValue(width / 2000);
      needsRender = true;
    }
    window.StudioScene = {
      set(joints, width) { values = joints; opening = width; apply(values,opening); },
      setCollisionLinks(names) {
        collisionLinks = new Set(names);
        for (const [name, link] of Object.entries(robot?.links || {})) link.traverse(child => {
          if (!child.isMesh) return;
          child.material.color.set(collisionLinks.has(name) ? '#e53935' : '#d8c9ad');
        });
        needsRender = true;
      },
      zoom(delta) { radius = Math.max(.4, Math.min(2.3, radius + delta)); cameraPosition(); },
      reset() { theta=1.1; phi=1.15; radius=1.05; cameraPosition(); }
    };
    renderer.domElement.addEventListener('pointerdown', e => { pointer=[e.clientX,e.clientY]; renderer.domElement.setPointerCapture(e.pointerId); });
    renderer.domElement.addEventListener('pointermove', e => {
      if (!pointer) return;
      theta -= (e.clientX-pointer[0])*.007; phi=Math.max(.2,Math.min(2.7,phi+(e.clientY-pointer[1])*.007));
      pointer=[e.clientX,e.clientY]; cameraPosition();
    });
    renderer.domElement.addEventListener('pointerup', () => { pointer=null; });
    renderer.domElement.addEventListener('pointercancel', () => { pointer=null; });
    renderer.domElement.addEventListener('wheel', e => { e.preventDefault(); radius=Math.max(.4,Math.min(2.3,radius+e.deltaY*.001)); cameraPosition(); }, { passive:false });
    new ResizeObserver(() => {
      const {width,height}=host.getBoundingClientRect();
      if(!width || !height)return;
      renderer.setSize(width,height); camera.aspect=width/height; camera.updateProjectionMatrix();
      needsRender = true;
    }).observe(host);
    renderer.setAnimationLoop(() => {if(needsRender){renderer.render(scene,camera);needsRender=false;}});
    document.getElementById('zoom-in').onclick = () => window.StudioScene.zoom(-.15);
    document.getElementById('zoom-out').onclick = () => window.StudioScene.zoom(.15);
    document.getElementById('expand-preview').onclick = async () => {
      try {
        if (document.fullscreenElement) await document.exitFullscreen();
        else await document.querySelector('.preview-panel').requestFullscreen();
      } catch (error) { status.textContent = `无法放大预览：${error.message}`; status.hidden = false; }
    };
    document.getElementById('reset-view').onclick = () => window.StudioScene.reset();
  } catch(error) {
    status.textContent = `3D 预览不可用：${error.message}`;
    window.StudioScene = {set() {}, setCollisionLinks() {}, zoom() {}, reset() {}};
  }
})();
