function followLegacyBookmark() {
  if (location.hash.startsWith("#/")) {
    location.replace(`/app/${location.hash}`);
  }
}
window.addEventListener("hashchange", followLegacyBookmark);
followLegacyBookmark();

const slider = document.querySelector("#detune");
const cents = document.querySelector("#cents-value");
const direction = document.querySelector("#direction-value");
const demo = document.querySelector("[data-strobe-demo]");

function updateStrobe() {
  const value = Number(slider.value);
  const still = Math.abs(value) < 0.05;
  cents.value = `${value > 0 ? "+" : value < 0 ? "−" : ""}${Math.abs(value).toFixed(1)} ¢`;
  direction.textContent = still
    ? "In tune · still"
    : value < 0
      ? "Flat · moving left"
      : "Sharp · moving right";
  slider.setAttribute(
    "aria-valuetext",
    `${Math.abs(value).toFixed(1)} cents ${still ? "in tune" : value < 0 ? "flat" : "sharp"}`,
  );
  demo.style.setProperty("--drift-state", still ? "paused" : "running");
  demo.style.setProperty("--drift-direction", value < 0 ? "reverse" : "normal");
  demo.style.setProperty(
    "--drift-duration",
    `${Math.max(0.35, 3 / Math.max(0.1, Math.abs(value)))}s`,
  );
}
slider.addEventListener("input", updateStrobe);
document.querySelector("#reset-detune").addEventListener("click", () => {
  slider.value = "0";
  updateStrobe();
});
updateStrobe();

// Older web installs had a root-scoped Workbox worker. Updating only that
// registration retires its cached app shell; the new /app/ worker stays intact.
if ("serviceWorker" in navigator) {
  navigator.serviceWorker
    .getRegistrations()
    .then((registrations) => {
      for (const registration of registrations) {
        if (registration.scope === `${location.origin}/`) {
          void registration.update().catch(() => {});
        }
      }
    })
    .catch(() => {});
}
