import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { RectAreaLightUniformsLib } from 'three/addons/lights/RectAreaLightUniformsLib.js';
import { ConvexGeometry } from 'three/addons/geometries/ConvexGeometry.js';
import { createTeachingView } from './teaching-view.js';

const GEOM = {
  PLANE: 0,
  HFIELD: 1,
  SPHERE: 2,
  CAPSULE: 3,
  ELLIPSOID: 4,
  CYLINDER: 5,
  BOX: 6,
  MESH: 7
};

const MAIN_VIEW_HELPER_LAYER = 1;
const MAX_CONTACT_VISUALS = 40;
const MOVABLE_OBJECT_BODY_NAMES = new Set(['red_cube', 'blue_block', 'yellow_cylinder']);

function partKind(name) {
  const value = name.toLowerCase();
  if (value.includes('d405') || value.includes('camera') || value.includes('imager')) return 'camera';
  if (value.includes('wordmark') || value.includes('badge')) return 'badge';
  if (value.includes('gripper') || value.includes('finger')) return 'gripper';
  if (value.includes('motor')) return 'motor';
  if (value.includes('cnc')) return 'structure';
  if (value.includes('pla')) return 'cover';
  if (value.includes('base')) return 'base';
  if (value.includes('link')) return 'link';
  return 'component';
}

function partOrder(bodyName) {
  if (bodyName.startsWith('gripper')) return 0;
  const link = bodyName.match(/^link([1-6])$/)?.[1];
  return link ? 7 - Number(link) : 7;
}

function namedObject(mujoco, model, type, id, fallback = '') {
  if (!Number.isInteger(id) || id < 0 || type == null) return fallback;
  return mujoco.mj_id2name(model, type, id) || fallback;
}

function nearestJointName(mujoco, model, bodyId) {
  let current = bodyId;
  while (current > 0) {
    const count = model.body_jntnum ? model.body_jntnum[current] : 0;
    if (count > 0) {
      const jointId = model.body_jntadr[current];
      return namedObject(mujoco, model, mujoco.mjtObj.mjOBJ_JOINT.value, jointId, `joint-${jointId}`);
    }
    current = model.body_parentid ? model.body_parentid[current] : 0;
  }
  return '';
}

function enumValue(mujoco, name, fallback) {
  const value = mujoco.mjtGeom?.[name]?.value;
  return Number.isInteger(value) ? value : fallback;
}

function geomTypes(mujoco) {
  return {
    plane: enumValue(mujoco, 'mjGEOM_PLANE', GEOM.PLANE),
    sphere: enumValue(mujoco, 'mjGEOM_SPHERE', GEOM.SPHERE),
    capsule: enumValue(mujoco, 'mjGEOM_CAPSULE', GEOM.CAPSULE),
    ellipsoid: enumValue(mujoco, 'mjGEOM_ELLIPSOID', GEOM.ELLIPSOID),
    cylinder: enumValue(mujoco, 'mjGEOM_CYLINDER', GEOM.CYLINDER),
    box: enumValue(mujoco, 'mjGEOM_BOX', GEOM.BOX),
    mesh: enumValue(mujoco, 'mjGEOM_MESH', GEOM.MESH)
  };
}

function copyFloats(source, start, count) {
  return Float32Array.from(source.subarray(start, start + count));
}

function createMeshGeometry(model, meshId) {
  const vertadr = model.mesh_vertadr[meshId];
  const vertnum = model.mesh_vertnum[meshId];
  const faceadr = model.mesh_faceadr[meshId];
  const facenum = model.mesh_facenum[meshId];
  if (!vertnum || !facenum) return new THREE.BufferGeometry();

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute(
    'position',
    new THREE.BufferAttribute(copyFloats(model.mesh_vert, vertadr * 3, vertnum * 3), 3)
  );
  const indices = Array.from(model.mesh_face.subarray(faceadr * 3, (faceadr + facenum) * 3));
  geometry.setIndex(indices);

  if (model.mesh_normal && model.mesh_normaladr && model.mesh_normalnum) {
    const normaladr = model.mesh_normaladr[meshId];
    const normalnum = model.mesh_normalnum[meshId];
    if (normalnum === vertnum) {
      geometry.setAttribute(
        'normal',
        new THREE.BufferAttribute(copyFloats(model.mesh_normal, normaladr * 3, normalnum * 3), 3)
      );
    }
  }
  if (!geometry.getAttribute('normal')) geometry.computeVertexNormals();
  geometry.computeBoundingSphere();
  return geometry;
}

function primitiveGeometry(type, size, types) {
  if (type === types.plane) {
    return new THREE.PlaneGeometry(2 * (size[0] || 5), 2 * (size[1] || 5));
  }
  if (type === types.sphere) {
    return new THREE.SphereGeometry(size[0], 24, 16);
  }
  if (type === types.capsule) {
    const geometry = new THREE.CapsuleGeometry(size[0], 2 * size[1], 8, 16);
    geometry.rotateX(Math.PI / 2);
    return geometry;
  }
  if (type === types.ellipsoid) {
    const geometry = new THREE.SphereGeometry(1, 24, 16);
    geometry.scale(size[0], size[1], size[2]);
    return geometry;
  }
  if (type === types.cylinder) {
    const geometry = new THREE.CylinderGeometry(size[0], size[0], 2 * size[1], 24);
    geometry.rotateX(Math.PI / 2);
    return geometry;
  }
  if (type === types.box) {
    return new THREE.BoxGeometry(2 * size[0], 2 * size[1], 2 * size[2]);
  }
  return null;
}

function geomMaterial(model, index, materialProps) {
  const rgba = model.geom_rgba.subarray(index * 4, index * 4 + 4);
  let color = new THREE.Color(rgba[0], rgba[1], rgba[2]);
  let opacity = rgba[3];
  let metalness = 0.12;
  let roughness = 0.55;
  const matid = model.geom_matid ? model.geom_matid[index] : -1;
  if (matid >= 0) {
    if (model.mat_rgba) {
      const matRgba = model.mat_rgba.subarray(matid * 4, matid * 4 + 4);
      color = new THREE.Color(matRgba[0], matRgba[1], matRgba[2]);
      opacity = matRgba[3];
    }
    const mp = materialProps && materialProps[matid];
    if (mp && (mp.metallic != null || mp.roughness != null)) {
      if (mp.metallic != null) metalness = mp.metallic;
      if (mp.roughness != null) roughness = mp.roughness;
    } else {
      if (model.mat_metallic) {
        metalness = model.mat_metallic[matid];
      } else if (model.mat_specular) {
        const spec = model.mat_specular.subarray(matid * 4, matid * 4 + 4);
        metalness = (spec[0] + spec[1] + spec[2]) / 3;
      }
      if (model.mat_roughness) {
        roughness = model.mat_roughness[matid];
      } else if (model.mat_shininess) {
        roughness = 1 - model.mat_shininess[matid];
      }
    }
  }
  const finish = materialProps && materialProps[matid];
  const isCnc = finish?.name === 'rs_anodized_silver_mat';
  const isTable = finish?.name === 'rs_table';
  const isMetal = metalness >= 0.45;
  if (isCnc) color.multiplyScalar(0.72);
  if (isTable) { color = new THREE.Color(0xc1c3b9); roughness = .95; metalness = 0; }
  return new THREE.MeshPhysicalMaterial({
    color,
    metalness: Number.isFinite(metalness) ? Math.max(0, Math.min(1, metalness)) : 0.12,
    roughness: Number.isFinite(roughness) ? Math.max(0, Math.min(1, roughness)) : 0.55,
    envMapIntensity: isCnc ? 1.9 : isMetal ? 1.2 : 0.72,
    clearcoat: isCnc ? 0.25 : 0,
    clearcoatRoughness: isCnc ? 0.14 : 0.4,
    transparent: opacity < 0.999,
    opacity,
    side: THREE.DoubleSide
  });
}

function shouldDrawGeom(model, index) {
  const group = model.geom_group ? model.geom_group[index] : 0;
  if (group >= 3) return false;
  const alpha = model.geom_rgba[index * 4 + 3];
  if (model.body_mocapid[model.geom_bodyid[index]] >= 0) return true;
  if (alpha < 0.02) return false;
  return true;
}

function setPose(mesh, xpos, xmat, index) {
  mesh.matrixAutoUpdate = false;
  mesh.matrix.set(
    xmat[index * 9 + 0], xmat[index * 9 + 1], xmat[index * 9 + 2], xpos[index * 3 + 0],
    xmat[index * 9 + 3], xmat[index * 9 + 4], xmat[index * 9 + 5], xpos[index * 3 + 1],
    xmat[index * 9 + 6], xmat[index * 9 + 7], xmat[index * 9 + 8], xpos[index * 3 + 2],
    0, 0, 0, 1
  );
  mesh.matrixWorldNeedsUpdate = true;
}

function createGridTexture() {
  const canvas = document.createElement('canvas');
  canvas.width = canvas.height = 512;
  const context = canvas.getContext('2d');
  context.fillStyle = '#b7bdb7';
  context.fillRect(0, 0, 512, 512);
  context.strokeStyle = '#9ba49b';
  context.lineWidth = 1;
  context.strokeRect(.5, .5, 511, 511);
  const texture = new THREE.CanvasTexture(canvas);
  texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
  texture.repeat.set(20, 20);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

function createEnvironmentTexture() {
  const canvas = document.createElement('canvas');
  canvas.width = 512;
  canvas.height = 256;
  const context = canvas.getContext('2d');
  const gradient = context.createLinearGradient(0, 0, 0, 256);
  gradient.addColorStop(0, '#ffffff');
  gradient.addColorStop(.5, '#b6bab9');
  gradient.addColorStop(1, '#525a5a');
  context.fillStyle = gradient;
  context.fillRect(0, 0, 512, 256);
  context.fillStyle = '#ffffff';
  context.fillRect(80, 20, 80, 150);
  context.fillRect(340, 20, 120, 150);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.mapping = THREE.EquirectangularReflectionMapping;
  return texture;
}

export function createSceneView(host, options = {}) {
  const scene = new THREE.Scene();
  const teaching = createTeachingView(scene, MAIN_VIEW_HELPER_LAYER, setPose);
  let collisionHullsVisible = false;
  scene.background = new THREE.Color(0xc5cbc7);

  const camera = new THREE.PerspectiveCamera(45, 1, 0.02, options.far || 20);
  camera.up.set(0, 0, 1);
  camera.position.set(...(options.position || [0.85, -0.95, 0.62]));
  camera.layers.enable(MAIN_VIEW_HELPER_LAYER);
  const overheadCamera = new THREE.PerspectiveCamera(48, 4 / 3, 0.02, 4);
  overheadCamera.matrixAutoUpdate = false;
  const wristCamera = new THREE.PerspectiveCamera(68, 4 / 3, 0.02, 4);
  wristCamera.matrixAutoUpdate = false;
  const previewDocument = host.ownerDocument;
  const overheadCanvas = previewDocument.getElementById('overhead-camera-canvas');
  const wristCanvas = previewDocument.getElementById('wrist-camera-canvas');

  const renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = .8;
  if ('outputColorSpace' in renderer) renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.domElement.style.zIndex = '0';
  host.appendChild(renderer.domElement);
  ['callouts', 'drag-cluster'].forEach((id) => {
    const node = host.querySelector(`#${id}`);
    if (node) host.appendChild(node);
  });

  const ndc = new THREE.Vector3();
  const CAMERA_PREVIEW_INTERVAL_MS = 100;
  const overheadRenderer = overheadCanvas
    ? new THREE.WebGLRenderer({ canvas: overheadCanvas, antialias: true, powerPreference: 'low-power' })
    : null;
  const wristRenderer = wristCanvas
    ? new THREE.WebGLRenderer({ canvas: wristCanvas, antialias: true, powerPreference: 'low-power' })
    : null;
  [overheadRenderer, wristRenderer].forEach((previewRenderer) => {
    if (!previewRenderer) return;
    previewRenderer.setPixelRatio(1);
    previewRenderer.setSize(320, 240, false);
    previewRenderer.shadowMap.enabled = false;
    previewRenderer.toneMapping = THREE.ACESFilmicToneMapping;
    previewRenderer.toneMappingExposure = .8;
    if ('outputColorSpace' in previewRenderer) previewRenderer.outputColorSpace = THREE.SRGBColorSpace;
  });

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(...(options.target || [0.28, 0, 0.16]));
  controls.enableDamping = true;

  const pmrem = new THREE.PMREMGenerator(renderer);
  const environmentSource = createEnvironmentTexture();
  const environmentTexture = pmrem.fromEquirectangular(environmentSource).texture;
  scene.environment = environmentTexture;
  environmentSource.dispose();
  pmrem.dispose();

  RectAreaLightUniformsLib.init();
  scene.add(new THREE.AmbientLight(0xffffff, 0.12));
  scene.add(new THREE.HemisphereLight(0xffffff, 0x717b78, 0.5));

  const softbox = new THREE.RectAreaLight(0xffffff, 1.0, 1.6, 1.15);
  softbox.position.set(0.28, -0.58, 1.28);
  softbox.lookAt(0.25, 0, 0.16);
  scene.add(softbox);

  const key = new THREE.DirectionalLight(0xf2fbff, 1.0);
  key.position.set(1.2, 0.8, 2.0);
  key.castShadow = true;
  key.shadow.mapSize.set(1024, 1024);
  key.shadow.camera.near = 0.1;
  key.shadow.camera.far = 6;
  key.shadow.camera.left = -1.4;
  key.shadow.camera.right = 1.4;
  key.shadow.camera.top = 1.4;
  key.shadow.camera.bottom = -1.4;
  key.shadow.bias = -0.00012;
  key.shadow.normalBias = 0.02;
  scene.add(key);

  const side = new THREE.DirectionalLight(0xffffff, 0.45);
  side.position.set(-1, -0.5, 0.8);
  scene.add(side);

  const rim = new THREE.DirectionalLight(0xfff3df, 0.4);
  rim.position.set(-0.5, -1, 1.5);
  scene.add(rim);

  const tcpMarker = new THREE.Mesh(
    new THREE.SphereGeometry(0.024, 16, 12),
    new THREE.MeshBasicMaterial({
      color: 0x33d6b0,
      wireframe: true,
      transparent: true,
      opacity: 0.4
    })
  );
  tcpMarker.visible = false;
  tcpMarker.layers.set(MAIN_VIEW_HELPER_LAYER);
  scene.add(tcpMarker);

  const targetGhost = new THREE.Mesh(
    new THREE.SphereGeometry(0.018, 28, 18),
    new THREE.MeshBasicMaterial({ color: 0xf2a541, transparent: true, opacity: 0.85 })
  );
  targetGhost.visible = false;
  targetGhost.layers.set(MAIN_VIEW_HELPER_LAYER);
  scene.add(targetGhost);

  const dragErrorPositions = new Float32Array(6);
  const dragErrorPositionAttribute = new THREE.BufferAttribute(dragErrorPositions, 3);
  dragErrorPositionAttribute.setUsage(THREE.DynamicDrawUsage);
  const dragErrorGeometry = new THREE.BufferGeometry();
  dragErrorGeometry.setAttribute('position', dragErrorPositionAttribute);
  const dragErrorLine = new THREE.Line(
    dragErrorGeometry,
    new THREE.LineBasicMaterial({ color: 0xff6b5f, transparent: true, opacity: 0.82 })
  );
  dragErrorLine.visible = false;
  dragErrorLine.frustumCulled = false;
  dragErrorLine.layers.set(MAIN_VIEW_HELPER_LAYER);
  scene.add(dragErrorLine);

  const raycaster = new THREE.Raycaster();
  raycaster.layers.enable(MAIN_VIEW_HELPER_LAYER);
  const ndcMouse = new THREE.Vector2();
  const planeHit = new THREE.Vector3();
  const selectionBounds = new THREE.Box3();
  const selectionHelper = new THREE.Box3Helper(selectionBounds, 0x4defff);
  selectionHelper.visible = false;
  selectionHelper.layers.set(MAIN_VIEW_HELPER_LAYER);
  scene.add(selectionHelper);

  const contactPointPositions = new Float32Array(MAX_CONTACT_VISUALS * 3);
  const contactPointColors = new Float32Array(MAX_CONTACT_VISUALS * 3);
  const contactLinePositions = new Float32Array(MAX_CONTACT_VISUALS * 6);
  const contactLineColors = new Float32Array(MAX_CONTACT_VISUALS * 6);
  const contactPointPositionAttribute = new THREE.BufferAttribute(contactPointPositions, 3);
  const contactPointColorAttribute = new THREE.BufferAttribute(contactPointColors, 3);
  const contactLinePositionAttribute = new THREE.BufferAttribute(contactLinePositions, 3);
  const contactLineColorAttribute = new THREE.BufferAttribute(contactLineColors, 3);
  [
    contactPointPositionAttribute,
    contactPointColorAttribute,
    contactLinePositionAttribute,
    contactLineColorAttribute
  ].forEach((attribute) => attribute.setUsage(THREE.DynamicDrawUsage));
  const contactPointGeometry = new THREE.BufferGeometry();
  contactPointGeometry.setAttribute('position', contactPointPositionAttribute);
  contactPointGeometry.setAttribute('color', contactPointColorAttribute);
  contactPointGeometry.setDrawRange(0, 0);
  contactPointGeometry.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 10);
  const contactPoints = new THREE.Points(
    contactPointGeometry,
    new THREE.PointsMaterial({ size: 0.008, sizeAttenuation: true, vertexColors: true })
  );
  contactPoints.visible = false;
  contactPoints.layers.set(MAIN_VIEW_HELPER_LAYER);
  scene.add(contactPoints);

  const contactNormalGeometry = new THREE.BufferGeometry();
  contactNormalGeometry.setAttribute('position', contactLinePositionAttribute);
  contactNormalGeometry.setAttribute('color', contactLineColorAttribute);
  contactNormalGeometry.setDrawRange(0, 0);
  contactNormalGeometry.boundingSphere = new THREE.Sphere(new THREE.Vector3(), 10);
  const contactNormals = new THREE.LineSegments(
    contactNormalGeometry,
    new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.92 })
  );
  contactNormals.visible = false;
  contactNormals.layers.set(MAIN_VIEW_HELPER_LAYER);
  scene.add(contactNormals);

  const meshes = [];
  const movableObjectMeshes = [];
  const partMeshesList = [];
  const partCountsByBody = new Map();
  const cameraModelMeshes = [];
  const partMeshes = new Map();
  const parts = [];
  const meshGeometries = new Map();
  let gridTexture = null;
  let types = geomTypes({});
  let selectedMesh = null;
  let partIsolated = false;
  let selectionListener = null;
  let pickStart = null;
  let lastCameraPreviewAt = -Infinity;
  let overheadCameraId = -1;
  let wristCameraId = -1;
  let overheadCameraEnabled = false;
  let wristCameraEnabled = false;
  let cameraModelVisible = true;
  let objectInteraction = {
    enabled: false,
    selectedBodyName: '',
    pose: null,
    onSelect: null,
    onMove: null,
    onRotate: null
  };
  let objectGesture = null;
  let obstacleInteraction = { enabled: false };
  const objectGesturePlane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
  const objectGestureHit = new THREE.Vector3();
  const arcballCurrent = new THREE.Vector3();
  const arcballDelta = new THREE.Quaternion();
  const arcballWorldDelta = new THREE.Quaternion();
  const arcballCameraInverse = new THREE.Quaternion();
  const arcballResult = new THREE.Quaternion();
  const objectPulseColor = new THREE.Color(0x6feaff);

  function resize() {
    const width = Math.max(1, host.clientWidth);
    const height = Math.max(1, host.clientHeight);
    camera.aspect = width / height;
    // Preserve the horizontal framing on narrow screens so the arm stays visible.
    camera.fov = THREE.MathUtils.radToDeg(2 * Math.atan(Math.tan(Math.PI / 8) / Math.min(1, camera.aspect)));
    camera.updateProjectionMatrix();
    renderer.setSize(width, height, false);
  }

  function build(mujoco, model, materialProps) {
    clear();
    types = geomTypes(mujoco);
    for (let i = 0; i < model.ngeom; i += 1) {
      const collisionHull = model.geom_group[i] === 3;
      if (!collisionHull && !shouldDrawGeom(model, i)) continue;
      if (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM.value, i) === 'ik_target_sphere') continue;
      const type = model.geom_type[i];
      const size = model.geom_size.subarray(i * 3, i * 3 + 3);
      let geometry = null;
      let meshId = -1;
      if (type === types.mesh) {
        meshId = model.geom_dataid[i];
        const geometryKey = collisionHull ? `hull:${meshId}` : meshId;
        if (!meshGeometries.has(geometryKey)) {
          const geometry = createMeshGeometry(model, meshId);
          if (collisionHull) {
            const vertices = geometry.getAttribute('position');
            const hull = new ConvexGeometry(Array.from({ length: vertices.count }, (_, j) => new THREE.Vector3().fromBufferAttribute(vertices, j)));
            geometry.dispose(); meshGeometries.set(geometryKey, hull);
          } else meshGeometries.set(geometryKey, geometry);
        }
        geometry = meshGeometries.get(geometryKey);
      } else {
        geometry = primitiveGeometry(type, size, types);
      }
      if (!geometry) continue;
      let material;
      if (collisionHull) {
        material = new THREE.MeshBasicMaterial({ color: 0x146f86, wireframe: true, transparent: true, opacity: .65, depthWrite: false });
      } else if (type === types.plane) {
        if (!gridTexture) gridTexture = createGridTexture();
        material = new THREE.MeshStandardMaterial({
          map: gridTexture,
          color: 0xffffff,
          metalness: 0.1,
          roughness: 0.82,
          envMapIntensity: 0.4
        });
      } else {
        material = geomMaterial(model, i, materialProps);
      }
      const mesh = new THREE.Mesh(geometry, material);
      mesh.castShadow = type !== types.plane;
      mesh.receiveShadow = true;
      mesh.userData.geomIndex = i;
      mesh.userData.collisionHull = collisionHull;
      mesh.userData.baseOpacity = material.opacity;
      mesh.userData.baseTransparent = material.transparent;
      if (collisionHull) { mesh.layers.set(MAIN_VIEW_HELPER_LAYER); mesh.castShadow = false; mesh.visible = collisionHullsVisible; }
      if (model.body_mocapid[model.geom_bodyid[i]] >= 0) {
        mesh.userData.mocapModel = model;
        if (type === types.box) mesh.userData.initialHalfSize = Array.from(size);
      }
      const isPart = (model.geom_group ? model.geom_group[i] : 0) === 1;
      const bodyId = model.geom_bodyid ? model.geom_bodyid[i] : -1;
      mesh.userData.bodyName = bodyId >= 0
        ? (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY.value, bodyId) || `body-${bodyId}`)
        : `geom-${i}`;
      if (!geometry.boundingSphere) geometry.computeBoundingSphere();
      if (!geometry.boundingBox) geometry.computeBoundingBox();
      mesh.userData.localCenter = geometry.boundingSphere?.center.clone() || new THREE.Vector3();
      mesh.userData.localBounds = geometry.boundingBox?.clone() || new THREE.Box3();
      scene.add(mesh);
      meshes.push(mesh);
      if (MOVABLE_OBJECT_BODY_NAMES.has(mesh.userData.bodyName)) {
        mesh.userData.objectBaseEmissive = mesh.material.emissive?.clone() || new THREE.Color(0);
        mesh.userData.objectBaseEmissiveIntensity = mesh.material.emissiveIntensity || 0;
        movableObjectMeshes.push(mesh);
      }
      if (isPart) {
        partMeshesList.push(mesh);
        const partCount = (partCountsByBody.get(mesh.userData.bodyName) || 0) + 1;
        partCountsByBody.set(mesh.userData.bodyName, partCount);
        const meshName = namedObject(
          mujoco,
          model,
          mujoco.mjtObj.mjOBJ_MESH?.value,
          meshId,
          ''
        );
        const geomName = namedObject(
          mujoco,
          model,
          mujoco.mjtObj.mjOBJ_GEOM.value,
          i,
          ''
        );
        const materialId = model.geom_matid ? model.geom_matid[i] : -1;
        const label = meshName || geomName || `${mesh.userData.bodyName}-${partCount}`;
        const part = {
          id: String(i),
          label,
          body: mesh.userData.bodyName,
          joint: nearestJointName(mujoco, model, bodyId),
          material: namedObject(
            mujoco,
            model,
            mujoco.mjtObj.mjOBJ_MATERIAL.value,
            materialId,
            ''
          ),
          kind: partKind(label)
        };
        mesh.userData.part = part;
        mesh.userData.cameraModel = part.kind === 'camera';
        if (mesh.userData.cameraModel) cameraModelMeshes.push(mesh);
        mesh.userData.baseEmissive = mesh.material.emissive?.clone() || null;
        mesh.userData.baseEmissiveIntensity = mesh.material.emissiveIntensity || 0;
        parts.push(part);
        partMeshes.set(part.id, mesh);
      }
    }
    parts.sort((a, b) => partOrder(a.body) - partOrder(b.body) || a.label.localeCompare(b.label));
    teaching.build(mujoco, model);
    setCollisionHulls(collisionHullsVisible);
    const cameraType = mujoco.mjtObj.mjOBJ_CAMERA?.value;
    overheadCameraId = cameraType == null
      ? -1
      : mujoco.mj_name2id(model, cameraType, 'overhead_rgb');
    wristCameraId = cameraType == null
      ? -1
      : mujoco.mj_name2id(model, cameraType, 'wrist_rgb');
    for (const [previewCamera, cameraId] of [
      [overheadCamera, overheadCameraId],
      [wristCamera, wristCameraId]
    ]) {
      const modelFov = Number(model.cam_fovy?.[cameraId]);
      if (cameraId >= 0 && Number.isFinite(modelFov) && modelFov > 0) {
        previewCamera.fov = modelFov;
        previewCamera.updateProjectionMatrix();
      }
    }
  }

  function sync(data) {
    teaching.sync(data);
    const xpos = data.geom_xpos;
    const xmat = data.geom_xmat;
    meshes.forEach((mesh) => {
      setPose(mesh, xpos, xmat, mesh.userData.geomIndex);
      if (mesh.userData.mocapModel) {
        const initial = mesh.userData.initialHalfSize;
        if (initial) {
          const size = mesh.userData.mocapModel.geom_size;
          mesh.scale.set(...initial.map((v, i) => size[mesh.userData.geomIndex * 3 + i] / v));
          mesh.matrix.scale(mesh.scale);
        }
        mesh.visible = mesh.userData.mocapModel.geom_rgba[mesh.userData.geomIndex * 4 + 3] > 0;
        mesh.material.opacity = 1;
        mesh.material.transparent = false;
      }
    });

    if (selectedMesh) {
      selectionBounds.copy(selectedMesh.userData.localBounds).applyMatrix4(selectedMesh.matrix);
      selectionHelper.visible = selectedMesh.visible;
      selectionHelper.updateMatrixWorld(true);
    }
    if (data.cam_xpos && data.cam_xmat) {
      if (overheadCameraId >= 0) setPose(overheadCamera, data.cam_xpos, data.cam_xmat, overheadCameraId);
      if (wristCameraId >= 0) setPose(wristCamera, data.cam_xpos, data.cam_xmat, wristCameraId);
    }
  }

  function render() {
    const now = performance.now();
    const objectPulse = 0.5 + 0.5 * Math.sin(now * 0.006);
    movableObjectMeshes.forEach((mesh) => {
      if (!mesh.material.emissive) return;
      mesh.material.emissive.copy(mesh.userData.objectBaseEmissive);
      mesh.material.emissiveIntensity = mesh.userData.objectBaseEmissiveIntensity;
      if (
        objectInteraction.enabled &&
        mesh.userData.bodyName === objectInteraction.selectedBodyName
      ) {
        mesh.material.emissive.lerp(objectPulseColor, 0.38 + objectPulse * 0.28);
        mesh.material.emissiveIntensity = 0.42 + objectPulse * 0.72;
      }
    });
    controls.update();
    renderer.render(scene, camera);
    if (now - lastCameraPreviewAt >= CAMERA_PREVIEW_INTERVAL_MS &&
        (overheadCameraEnabled || wristCameraEnabled)) {
      if (overheadCameraEnabled && overheadRenderer) {
        overheadRenderer.render(scene, overheadCamera);
      }
      if (wristCameraEnabled && wristRenderer) {
        wristRenderer.render(scene, wristCamera);
      }
      lastCameraPreviewAt = now;
    }
  }

  function projectWorld(x, y, z) {
    ndc.set(x, y, z).project(camera);
    const width = Math.max(1, host.clientWidth);
    const height = Math.max(1, host.clientHeight);
    return {
      x: (ndc.x * 0.5 + 0.5) * width,
      y: (-ndc.y * 0.5 + 0.5) * height,
      visible: ndc.z > -1 && ndc.z < 1 && ndc.x > -1.35 && ndc.x < 1.35
    };
  }

  function intersectPlane(clientX, clientY, plane) {
    const rect = host.getBoundingClientRect();
    ndcMouse.set(
      ((clientX - rect.left) / rect.width) * 2 - 1,
      -((clientY - rect.top) / rect.height) * 2 + 1
    );
    raycaster.setFromCamera(ndcMouse, camera);
    return raycaster.ray.intersectPlane(plane, planeHit) ? planeHit.clone() : null;
  }

  function setOrbitEnabled(enabled) {
    controls.enabled = enabled;
  }

  function setObjectInteraction(options = {}) {
    objectInteraction = {
      enabled: Boolean(options.enabled),
      selectedBodyName: options.selectedBodyName || '',
      pose: options.pose ? { ...options.pose } : null,
      onSelect: options.onSelect || null,
      onMove: options.onMove || null,
      onRotate: options.onRotate || null
    };
    host.classList.toggle('object-edit-mode', objectInteraction.enabled);
    if (!objectInteraction.enabled && objectGesture?.kind !== 'obstacle') {
      finishObjectGesture();
    }
  }

  function setObstacleInteraction(options) {
    obstacleInteraction = options;
    if (!options.enabled && objectGesture?.kind === 'obstacle') finishObjectGesture();
  }

  function setOverheadCameraEnabled(enabled) {
    overheadCameraEnabled = Boolean(enabled && overheadCameraId >= 0 && overheadRenderer);
    if (overheadCameraEnabled) lastCameraPreviewAt = -Infinity;
    return overheadCameraEnabled;
  }

  function setWristCameraEnabled(enabled) {
    wristCameraEnabled = Boolean(enabled && wristCameraId >= 0 && wristRenderer);
    if (wristCameraEnabled) lastCameraPreviewAt = -Infinity;
    return wristCameraEnabled;
  }

  function restoreSelectionMaterial(mesh) {
    if (!mesh?.material?.emissive) return;
    if (mesh.userData.baseEmissive) mesh.material.emissive.copy(mesh.userData.baseEmissive);
    mesh.material.emissiveIntensity = mesh.userData.baseEmissiveIntensity;
  }

  function applySelectionMaterial(mesh) {
    if (!mesh?.material?.emissive) return;
    mesh.material.emissive.set(0x18d9ff);
    mesh.material.emissiveIntensity = 0.72;
  }

  function applyIsolation() {
    partMeshesList.forEach((mesh) => {
      const allowedByCameraToggle = cameraModelVisible || !mesh.userData.cameraModel;
      mesh.visible = allowedByCameraToggle && (!partIsolated || mesh === selectedMesh);
    });
    host.classList.toggle('part-isolated', partIsolated);
    selectionHelper.visible = Boolean(selectedMesh?.visible);
  }

  function setCollisionHulls(enabled) {
    collisionHullsVisible = Boolean(enabled);
    let count = 0;
    meshes.forEach(mesh => {
      if (mesh.userData.collisionHull) { mesh.visible = collisionHullsVisible; count++; }
      else if (mesh.userData.part) {
        mesh.material.opacity = enabled ? .18 : mesh.userData.baseOpacity;
        mesh.material.transparent = enabled || mesh.userData.baseTransparent;
        mesh.material.depthWrite = !enabled;
      }
    });
    return count;
  }

  function notifySelection() {
    selectionListener?.({
      part: selectedMesh?.userData.part || null,
      isolated: partIsolated
    });
  }

  function selectPart(id) {
    const next = partMeshes.get(String(id));
    if (!next) return false;
    if (selectedMesh !== next) {
      restoreSelectionMaterial(selectedMesh);
      selectedMesh = next;
      applySelectionMaterial(selectedMesh);
    }
    applyIsolation();
    notifySelection();
    return true;
  }

  function setPartIsolated(enabled) {
    partIsolated = Boolean(enabled && selectedMesh);
    applyIsolation();
    notifySelection();
  }

  function clearPartSelection() {
    restoreSelectionMaterial(selectedMesh);
    selectedMesh = null;
    partIsolated = false;
    applyIsolation();
    selectionHelper.visible = false;
    notifySelection();
  }

  function setCameraModelVisible(enabled) {
    cameraModelVisible = Boolean(enabled);
    applyIsolation();
    return cameraModelVisible;
  }

  function pickPart(clientX, clientY) {
    const rect = renderer.domElement.getBoundingClientRect();
    ndcMouse.set(
      ((clientX - rect.left) / rect.width) * 2 - 1,
      -((clientY - rect.top) / rect.height) * 2 + 1
    );
    raycaster.setFromCamera(ndcMouse, camera);
    const hit = raycaster.intersectObjects(
      partMeshesList.filter((mesh) => mesh.visible),
      false
    )[0];
    if (hit) selectPart(hit.object.userData.part.id);
    else if (!partIsolated) clearPartSelection();
  }

  function setDragVisuals({ tcp, target, dragMode, dragging }) {
    tcpMarker.visible = Boolean(dragMode && tcp);
    if (tcp) tcpMarker.position.set(tcp.x, tcp.y, tcp.z);
    targetGhost.visible = Boolean(dragMode && dragging && target);
    if (target) targetGhost.position.set(target.x, target.y, target.z);
    const showLine = Boolean(dragMode && dragging && tcp && target && tcp.distanceTo(target) > 0.001);
    dragErrorLine.visible = showLine;
    if (showLine) {
      dragErrorPositions[0] = tcp.x;
      dragErrorPositions[1] = tcp.y;
      dragErrorPositions[2] = tcp.z;
      dragErrorPositions[3] = target.x;
      dragErrorPositions[4] = target.y;
      dragErrorPositions[5] = target.z;
      dragErrorPositionAttribute.needsUpdate = true;
    }
  }

  function setContactVisuals(contacts = [], enabled = true) {
    const count = enabled ? Math.min(contacts.length, MAX_CONTACT_VISUALS) : 0;
    if (!count) {
      contactPointGeometry.setDrawRange(0, 0);
      contactNormalGeometry.setDrawRange(0, 0);
      contactPoints.visible = false;
      contactNormals.visible = false;
      return;
    }
    for (let index = 0; index < count; index += 1) {
      const contact = contacts[index];
      const [x, y, z] = contact.position;
      const [nx, ny, nz] = contact.normal;
      const length = 0.012 + Math.min(0.038, Math.log1p(contact.force) * 0.007);
      const pointOffset = index * 3;
      contactPointPositions[pointOffset] = x;
      contactPointPositions[pointOffset + 1] = y;
      contactPointPositions[pointOffset + 2] = z;
      contactPointColors[pointOffset] = 1;
      contactPointColors[pointOffset + 1] = contact.selectedGrip ? 0.72 : 0.20;
      contactPointColors[pointOffset + 2] = contact.selectedGrip ? 0.10 : 0.18;

      const lineOffset = index * 6;
      contactLinePositions[lineOffset] = x;
      contactLinePositions[lineOffset + 1] = y;
      contactLinePositions[lineOffset + 2] = z;
      contactLinePositions[lineOffset + 3] = x + nx * length;
      contactLinePositions[lineOffset + 4] = y + ny * length;
      contactLinePositions[lineOffset + 5] = z + nz * length;
      const red = contact.selectedGrip ? 1 : 0.12;
      const green = contact.selectedGrip ? 0.78 : 0.92;
      const blue = contact.selectedGrip ? 0.16 : 1;
      contactLineColors[lineOffset] = red;
      contactLineColors[lineOffset + 1] = green;
      contactLineColors[lineOffset + 2] = blue;
      contactLineColors[lineOffset + 3] = red;
      contactLineColors[lineOffset + 4] = green;
      contactLineColors[lineOffset + 5] = blue;
    }
    contactPointPositionAttribute.needsUpdate = true;
    contactPointColorAttribute.needsUpdate = true;
    contactLinePositionAttribute.needsUpdate = true;
    contactLineColorAttribute.needsUpdate = true;
    contactPointGeometry.setDrawRange(0, count);
    contactNormalGeometry.setDrawRange(0, count * 2);
    contactPoints.visible = true;
    contactNormals.visible = true;
  }

  function clear() {
    restoreSelectionMaterial(selectedMesh);
    selectedMesh = null;
    partIsolated = false;
    selectionHelper.visible = false;
    host.classList.remove('part-isolated');
    meshes.forEach((mesh) => {
      scene.remove(mesh);
      mesh.material.dispose();
    });
    meshes.length = 0;
    movableObjectMeshes.length = 0;
    partMeshesList.length = 0;
    partCountsByBody.clear();
    cameraModelMeshes.length = 0;
    parts.length = 0;
    partMeshes.clear();
    meshGeometries.forEach((geometry) => geometry.dispose());
    meshGeometries.clear();
  }

  function updatePointerRay(clientX, clientY) {
    const rect = renderer.domElement.getBoundingClientRect();
    ndcMouse.set(
      ((clientX - rect.left) / rect.width) * 2 - 1,
      -((clientY - rect.top) / rect.height) * 2 + 1
    );
    raycaster.setFromCamera(ndcMouse, camera);
  }

  function projectToArcball(clientX, clientY, gesture, target) {
    let x = (clientX - gesture.centerClientX) / gesture.radius;
    let y = (gesture.centerClientY - clientY) / gesture.radius;
    const lengthSquared = x * x + y * y;
    if (lengthSquared > 1) {
      const inverseLength = 1 / Math.sqrt(lengthSquared);
      x *= inverseLength;
      y *= inverseLength;
      target.set(x, y, 0);
    } else {
      target.set(x, y, Math.sqrt(1 - lengthSquared));
    }
    return target;
  }

  function beginObjectGesture(event) {
    if (objectGesture) return true;
    const rotate = event.button === 2 || (event.button === 0 && (event.shiftKey || event.altKey));
    if ((!objectInteraction.enabled && !obstacleInteraction.enabled) || (event.button !== 0 && !rotate)) return false;
    updatePointerRay(event.clientX, event.clientY);
    const objectHit = raycaster.intersectObjects(
      meshes.filter(mesh => mesh.visible && (
        (objectInteraction.enabled && MOVABLE_OBJECT_BODY_NAMES.has(mesh.userData.bodyName)) ||
        (obstacleInteraction.enabled && mesh.userData.bodyName === 'task_obstacle'))),
      false
    )[0];
    if (!objectHit) return false;
    const bodyName = objectHit.object.userData.bodyName;
    const obstacle = bodyName === 'task_obstacle';
    if (obstacle && rotate) return false;
    if (!obstacle) objectInteraction.onSelect?.(bodyName);
    const interaction = obstacle ? obstacleInteraction : objectInteraction;
    const pose = interaction.pose;
    if (!pose) return true;
    if (rotate) {
      const screen = projectWorld(pose.x, pose.y, pose.z);
      const rect = renderer.domElement.getBoundingClientRect();
      const gesture = {
        mode: 'rotate',
        pointerId: event.pointerId,
        centerClientX: rect.left + screen.x,
        centerClientY: rect.top + screen.y,
        radius: Math.max(55, Math.min(110, rect.height * 0.14)),
        startVector: new THREE.Vector3(),
        cameraQuaternion: camera.getWorldQuaternion(new THREE.Quaternion()),
        startQuaternion: new THREE.Quaternion(
          pose.quaternion?.x || 0,
          pose.quaternion?.y || 0,
          pose.quaternion?.z || 0,
          pose.quaternion?.w ?? 1
        )
      };
      projectToArcball(event.clientX, event.clientY, gesture, gesture.startVector);
      objectGesture = gesture;
    } else {
      objectGesturePlane.constant = -pose.z;
      if (!raycaster.ray.intersectPlane(objectGesturePlane, objectGestureHit)) return true;
      objectGesture = {
        mode: 'move',
        pointerId: event.pointerId,
        offsetX: pose.x - objectGestureHit.x,
        offsetY: pose.y - objectGestureHit.y
      };
    }
    objectGesture.kind = obstacle ? 'obstacle' : 'object';
    objectGesture.onMove = interaction.onMove;
    objectGesture.onRotate = interaction.onRotate;
    interaction.onStart?.();
    controls.enabled = false;
    renderer.domElement.setPointerCapture?.(event.pointerId);
    host.classList.add(objectGesture.mode === 'move' ? 'object-dragging' : 'object-rotating');
    event.preventDefault();
    return true;
  }

  function moveObjectGesture(event) {
    if (!objectGesture || event.pointerId !== objectGesture.pointerId) return;
    if (objectGesture.mode === 'move') {
      updatePointerRay(event.clientX, event.clientY);
      if (!raycaster.ray.intersectPlane(objectGesturePlane, objectGestureHit)) return;
      objectGesture.onMove?.({
        x: objectGestureHit.x + objectGesture.offsetX,
        y: objectGestureHit.y + objectGesture.offsetY
      });
    } else {
      projectToArcball(event.clientX, event.clientY, objectGesture, arcballCurrent);
      arcballDelta.setFromUnitVectors(objectGesture.startVector, arcballCurrent);
      arcballCameraInverse.copy(objectGesture.cameraQuaternion).invert();
      arcballWorldDelta
        .copy(objectGesture.cameraQuaternion)
        .multiply(arcballDelta)
        .multiply(arcballCameraInverse);
      arcballResult
        .copy(arcballWorldDelta)
        .multiply(objectGesture.startQuaternion)
        .normalize();
      objectGesture.onRotate?.({
        w: arcballResult.w,
        x: arcballResult.x,
        y: arcballResult.y,
        z: arcballResult.z
      });
    }
    event.preventDefault();
  }

  function finishObjectGesture(event) {
    if (event && objectGesture && event.pointerId !== objectGesture.pointerId) return false;
    const handled = Boolean(objectGesture);
    if (!handled) return false;
    const pointerId = objectGesture.pointerId;
    objectGesture = null;
    if (renderer.domElement.hasPointerCapture?.(pointerId)) renderer.domElement.releasePointerCapture(pointerId);
    controls.enabled = true;
    host.classList.remove('object-dragging', 'object-rotating');
    return true;
  }

  function onPickPointerDown(event) {
    if (beginObjectGesture(event)) {
      pickStart = null;
      event.stopImmediatePropagation();
      return;
    }
    if (event.button !== 0) return;
    pickStart = { x: event.clientX, y: event.clientY, id: event.pointerId };
  }

  function onPickPointerUp(event) {
    if (finishObjectGesture(event)) return;
    if (!pickStart || event.pointerId !== pickStart.id) return;
    const distance = Math.hypot(event.clientX - pickStart.x, event.clientY - pickStart.y);
    pickStart = null;
    if (distance <= 5 && !host.classList.contains('tcp-dragging')) {
      pickPart(event.clientX, event.clientY);
    }
  }

  function onSelectionKeyDown(event) {
    if (event.key === 'Escape' && selectedMesh) clearPartSelection();
  }

  function onPickPointerCancel() {
    finishObjectGesture();
    pickStart = null;
  }

  function onContextMenu(event) {
    if (objectInteraction.enabled) event.preventDefault();
  }

  renderer.domElement.addEventListener('pointerdown', onPickPointerDown, true);
  renderer.domElement.addEventListener('pointermove', moveObjectGesture);
  renderer.domElement.addEventListener('pointerup', onPickPointerUp);
  renderer.domElement.addEventListener('pointercancel', onPickPointerCancel);
  renderer.domElement.addEventListener('lostpointercapture', onPickPointerCancel);
  renderer.domElement.addEventListener('contextmenu', onContextMenu);
  window.addEventListener('keydown', onSelectionKeyDown);
  window.addEventListener('blur', onPickPointerCancel);

  const observer = new ResizeObserver(resize);
  observer.observe(host);
  resize();

  return {
    build,
    sync,
    render,
    resize,
    projectWorld,
    intersectPlane,
    setObjectInteraction,
    setObstacleInteraction,
    setOrbitEnabled,
    setOverheadCameraEnabled,
    isOverheadCameraEnabled() {
      return overheadCameraEnabled;
    },
    hasOverheadCamera() {
      return overheadCameraId >= 0;
    },
    setWristCameraEnabled,
    isWristCameraEnabled() {
      return wristCameraEnabled;
    },
    hasWristCamera() {
      return wristCameraId >= 0;
    },
    setCameraModelVisible,
    isCameraModelVisible() {
      return cameraModelVisible;
    },
    hasCameraModel() {
      return cameraModelMeshes.length > 0;
    },
    getParts() {
      return parts.map((part) => ({ ...part }));
    },
    selectPart,
    setPartIsolated,
    clearPartSelection,
    onPartSelectionChange(listener) {
      selectionListener = listener;
      notifySelection();
    },
    setDragVisuals,
    setContactVisuals,
    setTeachingFrames: teaching.setFrames,
    setCollisionHulls,
    setPlanningTrace: teaching.setTrace,
    clearPlanningTrace: teaching.clearTrace,
    setTrajectoriesVisible: teaching.setTrajectoriesVisible,
    setProximityVisual: teaching.setProximity,
    setProximityVisible: teaching.setProximityVisible,
    camera,
    dispose() {
      teaching.dispose();
      observer.disconnect();
      renderer.domElement.removeEventListener('pointerdown', onPickPointerDown, true);
      renderer.domElement.removeEventListener('pointermove', moveObjectGesture);
      renderer.domElement.removeEventListener('pointerup', onPickPointerUp);
      renderer.domElement.removeEventListener('pointercancel', onPickPointerCancel);
      renderer.domElement.removeEventListener('lostpointercapture', onPickPointerCancel);
      renderer.domElement.removeEventListener('contextmenu', onContextMenu);
      window.removeEventListener('keydown', onSelectionKeyDown);
      window.removeEventListener('blur', onPickPointerCancel);
      clear();
      dragErrorGeometry.dispose();
      dragErrorLine.material.dispose();
      selectionHelper.geometry.dispose();
      selectionHelper.material.dispose();
      contactPointGeometry.dispose();
      contactPoints.material.dispose();
      contactNormalGeometry.dispose();
      contactNormals.material.dispose();
      overheadRenderer?.dispose();
      wristRenderer?.dispose();
      if (gridTexture) { gridTexture.dispose(); gridTexture = null; }
      environmentTexture.dispose();
      controls.dispose();
      renderer.dispose();
      renderer.domElement.remove();
    }
  };
}
