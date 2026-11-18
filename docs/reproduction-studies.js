(() => {
  const videos = [...document.querySelectorAll('.video-grid video')];
  const status = document.querySelector('#playback-status');
  const play = document.querySelector('#play-all');
  const pause = document.querySelector('#pause-all');
  let playbackRequest = 0;
  if (!videos.length || !status || !play || !pause) return;

  function pauseAll() {
    playbackRequest += 1;
    videos.forEach(video => video.pause());
    status.textContent = '三段视频已暂停。';
  }

  play.addEventListener('click', async () => {
    const request = ++playbackRequest;
    play.disabled = true;
    status.textContent = '正在载入三段视频…';
    try {
      await Promise.all(videos.map(video => new Promise((resolve, reject) => {
        video.pause();
        if (video.readyState >= 1) return resolve();
        const ready = () => { clean(); resolve(); };
        const fail = () => { clean(); reject(new Error('video load failed')); };
        const timer = setTimeout(fail, 15000);
        const clean = () => { clearTimeout(timer); video.removeEventListener('loadedmetadata', ready); video.removeEventListener('error', fail); };
        video.addEventListener('loadedmetadata', ready);
        video.addEventListener('error', fail);
        video.load();
      })));
      if (request !== playbackRequest) return;
      videos.forEach(video => { video.currentTime = 0; });
      await Promise.all(videos.map(video => video.play()));
      if (request !== playbackRequest) { videos.forEach(video => video.pause()); return; }
      status.textContent = '从 0 s 同步播放；各视频仍可独立控制。';
    } catch {
      videos.forEach(video => video.pause());
      status.textContent = '同步播放未能完成，请使用各视频的播放控件或下载视频。';
    } finally {
      play.disabled = false;
    }
  });
  pause.addEventListener('click', pauseAll);
})();
