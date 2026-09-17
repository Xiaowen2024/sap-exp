(function () {
  const navToggle = document.querySelector('.nav-toggle');
  const nav = document.querySelector('.site-nav');

  if (navToggle && nav) {
    navToggle.addEventListener('click', function () {
      const open = nav.classList.toggle('is-open');
      navToggle.setAttribute('aria-expanded', String(open));
      navToggle.textContent = open ? 'Close' : 'Menu';
    });
    nav.querySelectorAll('a').forEach(function (link) {
      link.addEventListener('click', function () {
        nav.classList.remove('is-open');
        navToggle.setAttribute('aria-expanded', 'false');
        navToggle.textContent = 'Menu';
      });
    });
  }

  const filterButtons = document.querySelectorAll('.filter-button');
  const experimentCards = document.querySelectorAll('.experiment-card');
  filterButtons.forEach(function (button) {
    button.addEventListener('click', function () {
      const filter = button.dataset.filter;
      filterButtons.forEach(function (item) { item.classList.toggle('is-active', item === button); });
      experimentCards.forEach(function (card) {
        const visible = filter === 'all' || card.dataset.category === filter;
        card.classList.toggle('is-hidden', !visible);
      });
    });
  });

  function renderBars(id, rows, options) {
    const root = document.getElementById(id);
    if (!root) return;
    const opts = options || {};
    const values = rows.map(function (row) { return row.value; });
    const minValue = opts.minValue !== undefined ? opts.minValue : Math.min.apply(Math, values);
    const maxValue = opts.maxValue !== undefined ? opts.maxValue : Math.max.apply(Math, values);
    const useLog = Boolean(opts.log);
    const transform = function (value) { return useLog ? Math.log10(Math.max(value, 1e-12)) : value; };
    const low = transform(minValue);
    const high = transform(maxValue);
    const span = Math.max(high - low, 1e-9);
    root.innerHTML = rows.map(function (row) {
      const transformed = transform(row.value);
      let width;
      if (opts.zeroBased) {
        width = (transformed / high) * 100;
      } else {
        width = 12 + ((transformed - low) / span) * 88;
      }
      const tone = row.tone ? ' ' + row.tone : '';
      return '<div class="bar-row">' +
        '<span class="bar-label">' + row.label + '</span>' +
        '<span class="bar-track"><span class="bar-fill' + tone + '" style="width:' + width.toFixed(1) + '%"></span></span>' +
        '<span class="bar-value">' + row.display + '</span>' +
        '</div>';
    }).join('');
  }

  renderBars('rolling-chart', [
    { label: 'MuJoCo', value: 0.00692691, display: '0.00693' },
    { label: 'MuJoCo best', value: 0.00114589, display: '0.00115' },
    { label: 'SAP best', value: 0.34721228, display: '0.34721', tone: 'sap' }
  ], { log: true, minValue: 0.001 });

  renderBars('impact-chart', [
    { label: 'MuJoCo', value: 0.802717, display: '0.8027' },
    { label: 'SAP', value: 0.745222, display: '0.7452', tone: 'sap' },
    { label: 'target', value: 0.8, display: '0.8000', tone: 'target' }
  ], { minValue: 0.70, maxValue: 0.82 });

  renderBars('panda-chart', [
    { label: 'MuJoCo', value: 4, display: '4 / 46' },
    { label: 'SAP/Warp', value: 32, display: '32 / 46', tone: 'sap' }
  ], { minValue: 0, maxValue: 46, zeroBased: true });
})();
