/* ---------- routing, motion, boot ---------- */
EXTRA_LESSONS.forEach((x, i) => LESSONS.splice(x.at + i, 0, x.L));
GALLERY.push(
  { img: 's2_before', title: 'S2: welded body before the first crawl goal', shows: 'The robot fixed above the ground before the fixed base crawl attempt of 2026-10-30.', matters: 'The setup in which all crawl phase goals first ran on the real controller.', prov: 'Cloud run s2_phase_c_20261030T062140Z.', not: 'The attempt’s frozen verdict was FAIL. It was accepted later under revised criteria. No contact.', l: 'T' },
  { img: 's3_before', title: 'S3: at rest before the first crawl on the ground', shows: 'The free robot standing on the ground before the first goal of the crawl run.', matters: 'The starting point of the only kind of run in which the feet pushed against the ground.', prov: 'Cloud run s3_free_base_20261101T115002Z.', not: 'A still image shows no motion.', l: 'T' },
  { img: 's3_after', title: 'S3: after six crawl cycles', shows: 'The robot after the scripted six cycles and the estop hold. The committed scorer measured 0.1201 m of forward travel.', matters: 'Paired with the picture before, it is the only visual record of ground locomotion in the repository.', prov: 'Cloud run s3_free_base_20261101T115002Z.', not: 'Keyboard walking by a person, balance, or hardware. The run’s frozen verdict is INCONCLUSIVE.', l: 'T' });
for (let i = GALLERY.length - 1; i >= 0; i--) if (!DATA.images[GALLERY[i].img]) GALLERY.splice(i, 1);
const VIEWS = [['learn', 'Guided lessons', 'g'], ['code', 'Source code path', 'g'], ['mission', 'Mission Control', 'e'], ['robot', 'Explore the Robot', 'e'], ['description', 'Robot Description', 'e'], ['systems', 'Subsystems', 'e'], ['flow', 'Command to Motion', 'e'], ['map', 'ROS 2 System Map', 'e'], ['lab', 'Kinematics and Gait Lab', 'e'], ['evidence', 'Tests and Evidence', 'e'], ['milestones', 'Milestone Map', 'e'], ['next', 'Limits and Next', 'e']];
const navGroup = (g, title, sub) => '<span class="navh">' + title + '<small>' + sub + '</small></span>' + VIEWS.filter((v) => v[2] === g).map(([id, t]) => '<a href="#' + id + '">' + t + '</a>').join('');
$('#nav').innerHTML = navGroup('g', 'Guided learning', 'Start here, in order') + navGroup('e', 'Explore the system', 'Go anywhere');
const main = $('#main');
function route() {
  const id = (location.hash || '#mission').slice(1), v = R[id] ? id : 'mission'; state.view = v; state.playing = false;
  document.querySelectorAll('#nav a').forEach((a) => { if (a.getAttribute('href') === '#' + v) a.setAttribute('aria-current', 'page'); else a.removeAttribute('aria-current'); });
  if (stageEl.parentNode) stageEl.remove(); if (viewer.ok) { viewer.onPick = null; viewer.onFrame = null; viewer.colors = {}; viewer.hidden.clear(); viewer.autoRotate = false; }
  EV.length = 0; const el = document.createElement('section'); el.className = 'view'; el.id = 'v-' + v; main.replaceChildren(el); R[v](el); window.scrollTo(0, 0);
  const a = $('#nav a[aria-current]'); if (a && a.scrollIntoView && innerWidth < 981) a.scrollIntoView({ inline: 'center', block: 'nearest' });
}
addEventListener('hashchange', route);
const mb = $('#motionBtn');
function setMotion(on) { state.motion = on; document.documentElement.classList.toggle('still', !on); mb.setAttribute('aria-pressed', on); mb.innerHTML = on ? '❙❙ <span>Pause motion</span>' : '▶ <span>Resume motion</span>'; if (!on) { state.playing = false; state.flowPlay = false; } if (viewer.ok) viewer.autoRotate = on && state.view === 'mission'; }
mb.onclick = () => { setMotion(!state.motion); if (state.view === 'flow') route(); };
setMotion(state.motion); route();
(function boot() {
  const b = $('#boot'), end = () => { b.classList.add('out'); setTimeout(() => b.remove(), 500); };
  if (!state.motion || store.get('booted', false)) { b.remove(); return; }
  const c = DATA.robot.counts, lines = ['Reading snapshot ' + SNAP.short + ' of ' + SNAP.date, c.links + ' links, ' + c.joints + ' joints, ' + c.revolute + ' revolute', c.meshes + ' meshes, ' + c.triangles.toLocaleString('en') + ' triangles', 'Not connected to a robot or simulator'];
  lines.forEach((t, i) => { const li = document.createElement('li'); li.textContent = '> ' + t; li.style.animationDelay = (0.35 + i * 0.45) + 's'; $('#bootLines').appendChild(li); });
  const t = setTimeout(end, 3000); store.set('booted', true);
  const skip = () => { clearTimeout(t); end(); }; $('#bootSkip').onclick = skip; b.addEventListener('keydown', skip); $('#bootSkip').focus();
})();
