const MAX_VISIBLE_CONTACTS = 40;

function namedId(mujoco, model, type, name) {
  const id = mujoco.mj_name2id(model, type, name);
  if (id < 0) throw new Error(`找不到 ${name}`);
  return id;
}

export function createContactTelemetry(mujoco, model, data) {
  const bodyType = mujoco.mjtObj.mjOBJ_BODY.value;
  const fingerBodies = new Set([
    namedId(mujoco, model, bodyType, 'gripper_left'),
    namedId(mujoco, model, bodyType, 'gripper_right')
  ]);
  const objectBodies = {
    red: namedId(mujoco, model, bodyType, 'red_cube'),
    blue: namedId(mujoco, model, bodyType, 'blue_block'),
    yellow: namedId(mujoco, model, bodyType, 'yellow_cylinder')
  };
  const forceBuffer = new mujoco.DoubleBuffer(6);

  function sample(selectedId = 'red', includeVisuals = true) {
    const visible = [];
    const selectedBody = objectBodies[selectedId] ?? -1;
    let graspForce = 0;
    let activeContacts = 0;
    let peakForce = 0;
    const contacts = data.contact;
    const count = Math.min(data.ncon, contacts.size());
    try {
      for (let index = 0; index < count; index++) {
        const contact = contacts.get(index);
        try {
          if (!contact) continue;
          mujoco.mj_contactForce(model, data, index, forceBuffer);
          const normalForce = Math.abs(forceBuffer.GetView()[0]);
          if (!Number.isFinite(normalForce) || normalForce < .01) continue;
          activeContacts++;
          peakForce = Math.max(peakForce, normalForce);
          const body1 = model.geom_bodyid[contact.geom1];
          const body2 = model.geom_bodyid[contact.geom2];
          const selectedGrip =
            (fingerBodies.has(body1) && body2 === selectedBody) ||
            (fingerBodies.has(body2) && body1 === selectedBody);
          if (selectedGrip) graspForce += normalForce;
          if (includeVisuals && visible.length < MAX_VISIBLE_CONTACTS) {
            const direction = body2 === selectedBody ? 1 : -1;
            visible.push({
              position: Array.from(contact.pos),
              normal: [0, 1, 2].map(i => contact.frame[i] * direction),
              force: normalForce, selectedGrip,
            });
          }
        } finally { contact?.delete(); }
      }
    } finally { contacts.delete(); }

    return { contacts: visible, activeContacts, graspForce, peakForce };
  }

  return { sample };
}
