(() => {
  const toggle = document.querySelector(".mobile-nav-toggle");
  const nav = document.querySelector("#report-nav");
  const links = [...document.querySelectorAll(".report-nav a")];
  const sections = [...document.querySelectorAll("main section[id], main header[id]")];

  if (toggle && nav) {
    toggle.addEventListener("click", () => {
      const open = nav.classList.toggle("is-open");
      toggle.setAttribute("aria-expanded", String(open));
    });

    links.forEach((link) => {
      link.addEventListener("click", () => {
        nav.classList.remove("is-open");
        toggle.setAttribute("aria-expanded", "false");
      });
    });
  }

  if ("IntersectionObserver" in window) {
    const observer = new IntersectionObserver(
      (entries) => {
        entries.forEach((entry) => {
          if (!entry.isIntersecting) return;
          links.forEach((link) => {
            link.classList.toggle(
              "is-active",
              link.getAttribute("href") === "#" + entry.target.id
            );
          });
        });
      },
      { rootMargin: "-18% 0px -72% 0px", threshold: 0 }
    );
    sections.forEach((section) => observer.observe(section));
  }
})();
