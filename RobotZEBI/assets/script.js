const currentPage = document.body.dataset.page;

document.querySelectorAll(".nav-links a").forEach((link) => {
  const href = link.getAttribute("href") || "";
  if (href.includes(currentPage)) {
    link.classList.add("active");
  }
  if (currentPage === "index" && href === "index.html") {
    link.classList.add("active");
  }
});

const sensorDots = document.querySelectorAll(".sensor-dot");
let tick = 0;

function animateSensors() {
  sensorDots.forEach((dot, index) => {
    const active = Math.abs(index - (3.5 + Math.sin(tick / 18) * 2.2)) < 1.15;
    dot.style.background = active ? "#f5f5f5" : "#2b303a";
    dot.style.opacity = active ? "1" : "0.55";
  });
  tick += 1;
  requestAnimationFrame(animateSensors);
}

if (sensorDots.length) {
  animateSensors();
}
