// Migration worker for the former tuner at /. The new tuner is at /app/
// with its own service worker. Never intercept or cache the landing page.
self.addEventListener("install", () => self.skipWaiting());
self.addEventListener("activate", (event) => {
  event.waitUntil(
    (async () => {
      const oldPrecache = `workbox-precache-v2-${self.registration.scope}`;
      for (const key of await caches.keys()) {
        if (key === oldPrecache) await caches.delete(key);
      }
      await self.clients.claim();
      await self.registration.unregister();
    })(),
  );
});
