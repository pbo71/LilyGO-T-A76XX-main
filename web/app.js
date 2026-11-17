// Shared page code. Each page sets PAGE before loading this file:
//   channel: ThingSpeak channel ID (must be in config.php)
//   fields:  the fields to show, in card order: name (fallback when ThingSpeak has none), unit,
//            digits, optional scale, and daily: true for values sent once a day
//   charts:  title, fields and unit of each chart
const { channel: CHANNEL, fields: FIELDS, charts: CHARTS } = PAGE;
const TZ = 'Europe/Copenhagen';
const css = name => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const charts = [];
let names = {};

function label(f) { return (names && names[f]) || FIELDS[f].name; }
function value(f, v) { return v * (FIELDS[f].scale || 1); }
function fmt(f, v) {
  return value(f, v).toLocaleString('da-DK', { minimumFractionDigits: FIELDS[f].digits, maximumFractionDigits: FIELDS[f].digits });
}
function age(ms) {
  const min = Math.round((Date.now() - ms) / 60000);
  if (min < 1) return 'lige nu';
  if (min < 60) return `${min} min siden`;
  if (min < 48 * 60) return `${Math.round(min / 60)} t siden`;
  return `${Math.round(min / 1440)} dage siden`;
}
function timeText(ms, opts) { return new Date(ms).toLocaleString('da-DK', { timeZone: TZ, ...opts }); }

// A value is stale when the latest report did not include it (e.g. the Bluetooth sensor was not
// heard). Daily values (vDrop) are stale only after 36 hours.
function isStale(f, t, newest) {
  if (FIELDS[f].daily) return Date.now() - t > 36 * 3600e3;
  return t < newest - 10 * 60e3;
}

function renderCards(latest) {
  const el = document.getElementById('cards');
  el.innerHTML = '';
  const newest = Math.max(...Object.values(latest).filter(Boolean).map(l => l.t));
  for (const f of Object.keys(FIELDS)) {
    const l = latest[f];
    if (!l) continue;
    const card = document.createElement('div');
    card.className = isStale(f, l.t, newest) ? 'card stale' : 'card';
    if (card.className.includes('stale')) card.title = 'Ikke med i seneste måling';
    card.innerHTML = `<div class="label"></div><div class="value"></div><div class="age"></div>`;
    card.querySelector('.label').textContent = label(f);
    card.querySelector('.value').textContent = `${fmt(f, l.v)} ${FIELDS[f].unit}`;
    card.querySelector('.age').textContent = age(l.t);
    el.appendChild(card);
  }
}

// Offset of Danish time from UTC at time t, in ms (handles summer time)
function tzOffset(t) {
  const p = Object.fromEntries(new Intl.DateTimeFormat('en-US', {
    timeZone: TZ, hourCycle: 'h23', year: 'numeric', month: 'numeric', day: 'numeric',
    hour: 'numeric', minute: 'numeric', second: 'numeric',
  }).formatToParts(new Date(t)).map(x => [x.type, x.value]));
  return Date.UTC(p.year, p.month - 1, p.day, p.hour, p.minute, p.second) - Math.floor(t / 1000) * 1000;
}

// Axis span and ticks on whole hours, days or months in Danish time
const RANGE = {
  '24h': { span: 24 * 3600e3, step: 3 * 3600e3, label: { hour: '2-digit', minute: '2-digit' } },
  '7d':  { span: 7 * 86400e3, step: 86400e3, label: { weekday: 'short', day: 'numeric' } },
  '30d': { span: 30 * 86400e3, step: 5 * 86400e3, label: { day: 'numeric', month: 'short' } },
  '1y':  { span: 365 * 86400e3, step: 0, label: { month: 'short' } },  // step 0 = first of each month
};
function axisTicks(min, max, range) {
  const r = RANGE[range], ticks = [];
  if (r.step === 0) {
    const d = new Date(min + tzOffset(min));
    let y = d.getUTCFullYear(), m = d.getUTCMonth() + 1;
    for (;;) {
      if (m > 11) { y++; m = 0; }
      const local = Date.UTC(y, m, 1);
      const t = local - tzOffset(local);
      if (t > max) break;
      ticks.push(t);
      m++;
    }
    return ticks;
  }
  for (let local = Math.ceil((min + tzOffset(min)) / r.step) * r.step; ; local += r.step) {
    const t = local - tzOffset(local);
    if (t > max) break;
    if (t >= min) ticks.push(t);
  }
  return ticks;
}

// Where no value came for more than 3 hours (the night pause, or a sensor that was not heard),
// the line is drawn thin and dashed: there are no measurements in between
const GAP = 3 * 3600e3;
const isGap = ctx => ctx.p1.parsed.x - ctx.p0.parsed.x > GAP;
const gapSegment = { borderDash: ctx => isGap(ctx) ? [4, 4] : undefined, borderWidth: ctx => isGap(ctx) ? 1 : undefined };

function renderCharts(rows, range) {
  const host = document.getElementById('charts');
  charts.forEach(c => c.destroy());
  charts.length = 0;
  host.innerHTML = '';
  const colors = [css('--c1'), css('--c2'), css('--c3')];
  const xMax = Date.now(), xMin = xMax - RANGE[range].span;
  const ticks = axisTicks(xMin, xMax, range);

  for (const def of CHARTS) {
    const datasets = def.fields.map((f, i) => ({
      label: label(f),
      data: rows.filter(r => r[f] !== null).map(r => ({ x: r.t, y: value(f, r[f]) })),
      borderColor: colors[i], backgroundColor: colors[i],
      borderWidth: 2, pointRadius: rows.length > 200 ? 0 : 2, tension: 0,  // Straight lines: only measured values
      segment: gapSegment,
    })).filter(d => d.data.length > 0);
    if (datasets.length === 0) continue;

    const box = document.createElement('div');
    box.className = 'chart';
    box.innerHTML = `<h2></h2><div class="box"><canvas></canvas></div>`;
    box.querySelector('h2').textContent = def.title;
    host.appendChild(box);

    charts.push(new Chart(box.querySelector('canvas'), {
      type: 'line',
      data: { datasets },
      options: {
        maintainAspectRatio: false, animation: false,
        interaction: { mode: 'nearest', axis: 'x', intersect: false },
        scales: {
          x: { type: 'linear', min: xMin, max: xMax, grid: { color: css('--line') },
               afterBuildTicks: axis => { axis.ticks = ticks.map(value => ({ value })); },
               ticks: { color: css('--muted'), callback: v => timeText(v, RANGE[range].label) } },
          y: { grid: { color: css('--line') }, ticks: { color: css('--muted') },
               title: { display: true, text: def.unit, color: css('--muted') } },
        },
        plugins: {
          legend: { display: datasets.length > 1, labels: { color: css('--text'), boxWidth: 12 } },
          tooltip: { callbacks: {
            title: items => timeText(items[0].parsed.x, { dateStyle: 'medium', timeStyle: 'short' }),
            label: item => `${item.dataset.label}: ${item.parsed.y.toLocaleString('da-DK')} ${def.unit}`,
          } },
        },
      },
    }));
  }
  if (charts.length === 0) host.innerHTML = '<p class="msg">Ingen målinger i perioden.</p>';
}

async function load(range) {
  try {
    const res = await fetch(`api.php?channel=${CHANNEL}&range=${range}`);
    const data = await res.json();
    if (data.error) throw new Error(data.error);
    names = data.fields || {};
    if (data.channel) document.getElementById('title').textContent = data.channel;
    renderCards(data.latest);
    renderCharts(data.rows, range);
    const times = Object.values(data.latest).filter(Boolean).map(l => l.t);
    document.getElementById('updated').textContent = times.length
      ? `Seneste måling: ${timeText(Math.max(...times), { dateStyle: 'long', timeStyle: 'short' })}`
      : 'Ingen målinger endnu';
    const ver = data.latest.field1;
    document.getElementById('footer').textContent = ver ? `Firmware version ${ver.v}` : '';
  } catch (e) {
    document.getElementById('updated').textContent = 'Kunne ikke hente data. Prøv igen senere.';
  }
}

let current = '24h';
document.getElementById('ranges').addEventListener('click', e => {
  const b = e.target.closest('button');
  if (!b) return;
  current = b.dataset.range;
  document.querySelectorAll('#ranges button').forEach(x => x.classList.toggle('active', x === b));
  load(current);
});
load(current);
setInterval(() => load(current), 5 * 60 * 1000);  // The board reports hourly, so this is plenty
