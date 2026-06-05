import RobotWindow from 'https://cyberbotics.com/wwi/R2025a/RobotWindow.js';

const robotWindow = new RobotWindow();
robotWindow.setTitle('SS928 Control');

const fields = {
  connection: document.getElementById('connection'),
  mode: document.getElementById('mode'),
  speed: document.getElementById('speed'),
  angle: document.getElementById('angle'),
  yaw: document.getElementById('yaw'),
  distance: document.getElementById('distance'),
  position: document.getElementById('position'),
  log: document.getElementById('log')
};

function send(message) {
  robotWindow.send(message);
}

function updateState(state) {
  fields.connection.textContent = state.running ? 'Running' : 'Paused';
  fields.connection.classList.add('live');
  fields.mode.textContent = `${state.mode} / ${state.auto}`;
  fields.speed.textContent = state.speed;
  fields.angle.textContent = `${state.angle} deg`;
  fields.yaw.textContent = `${state.yaw} deg`;
  fields.distance.textContent = `${state.distance} cm`;
  fields.position.textContent = `${state.x} / ${state.y}`;
  fields.log.textContent = (state.log || []).join('\n');
}

robotWindow.receive = (message) => {
  try {
    const state = JSON.parse(message);
    if (state.type === 'state')
      updateState(state);
  } catch (_) {
    fields.log.textContent = message;
  }
};

window.addEventListener('load', () => {
  document.querySelectorAll('[data-command]').forEach((button) => {
    button.addEventListener('click', () => send(button.dataset.command));
  });

  document.querySelectorAll('[data-speed]').forEach((button) => {
    button.addEventListener('click', () => send(`speed:${button.dataset.speed}`));
  });

  send('camera');
});
