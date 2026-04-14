// GoTree2 SensorBox dashboard script.
// - Receives telemetry on `/ws/telemetry` (WebSocket).
// - Sends commands via POST endpoints (e.g. `/nav/goto_h`, `/actuator/stop`).
// - Can be served from SPIFFS or opened from `file://` during development.

const WS_FALLBACK_HOST = "192.168.4.1"; // SoftAP default; used when page is opened from file://
const WS_HOST_OVERRIDE_KEY = "gottree-ws-host";
const TELEMETRY_PATH = "/ws/telemetry";
const TELEMETRY_RETRY_MS = 2000;
const TELEMETRY_STALE_MS = 3000;
const CHECKUP_MIN_READY_DELAY_MS = 15000;

let telemetrySocket = null;
let telemetryLastMessageAt = 0;
let telemetryStaleWarned = false;

const ledExp = document.getElementById("ledExp");
const ledDac = document.getElementById("ledDac");
const decisionBox = document.getElementById("decisionBox");
const decisionText = document.getElementById("decisionText");
const decisionPhase = document.getElementById("decisionPhase");
const targetBox = document.getElementById("targetBox");
const targetText = document.getElementById("targetText");
const statusExp = document.getElementById("statusExp");
const statusDac = document.getElementById("statusDac");
const statusHdop = document.getElementById("statusHdop");
const statusSats = document.getElementById("statusSats");
const metricDistance = document.getElementById("metricDistance");
const metricHeading = document.getElementById("metricHeading");
const themeToggleBtn = document.getElementById("btnThemeToggle");

const btnGotoPost = document.getElementById("btnGotoPost");
const btnGotoH = document.getElementById("btnGotoH");
const btnStop = document.getElementById("btnStop");
const btnInitialCheckupRoutine = document.getElementById(
  "btnInitialCheckupRoutine"
);
const navButtons = {
  goto_post: btnGotoPost,
  goto_h: btnGotoH,
};

const telemetry = {
  decision: "STOP",
  phase: "",
  target: "",
  dist_m: null,
  heading_deg: null,
  exp_ready: null,
  dac_ready: null,
  gps_hdop: null,
  gps_sats: null,
};

let checkupInProgress = false;
let isSystemReady = false;
let checkupStartedAtMs = 0;
let checkupReadyTimeout = null;
let navActiveTarget = null;
let navLockedKey = null;
let stopFlashTimeout = null;
let lastStopFlashAt = 0;

const THEME_STORAGE_KEY = "gottree-dashboard-theme";
const rootEl = document.documentElement;

function applyTheme(theme) {
  const target = theme === "light" ? "light" : "dark";
  rootEl.setAttribute("data-theme", target);
  if (themeToggleBtn) {
    themeToggleBtn.textContent =
      target === "light" ? "Dark Mode" : "Light Mode";
  }
  try {
    localStorage.setItem(THEME_STORAGE_KEY, target);
  } catch (e) {
    /* ignore */
  }
}

const savedTheme = (() => {
  try {
    return localStorage.getItem(THEME_STORAGE_KEY);
  } catch (e) {
    return null;
  }
})();
applyTheme(savedTheme || "dark");

if (themeToggleBtn) {
  themeToggleBtn.addEventListener("click", () => {
    const next =
      rootEl.getAttribute("data-theme") === "light" ? "dark" : "light";
    applyTheme(next);
  });
}

function toBool(value) {
  if (value === undefined || value === null) return null;
  if (typeof value === "boolean") return value;
  if (typeof value === "number") return value !== 0;
  if (typeof value === "string") {
    const lower = value.trim().toLowerCase();
    if (lower === "true" || lower === "yes" || lower === "1") return true;
    if (lower === "false" || lower === "no" || lower === "0") return false;
  }
  return Boolean(value);
}

function setHeaderLed(el, ready) {
  if (!el) return;
  if (ready) {
    el.classList.add("led-ok");
    el.classList.remove("led-err");
    return;
  }
  el.classList.add("led-err");
  el.classList.remove("led-ok");
}

function setStatusIndicator(el, statusClass, text) {
  if (!el) return;
  el.classList.remove("status-ok", "status-warn", "status-err");
  if (statusClass) {
    el.classList.add(statusClass);
  }
  if (text !== undefined) {
    el.textContent = text;
  }
}

function updateDecision(decision, phase) {
  if (!decisionText || !decisionBox) return;
  const value = decision && String(decision).trim().length
    ? String(decision)
    : "IDLE";
  const normalized = value.trim().toUpperCase();
  const isStop =
    normalized === "STOP" ||
    normalized === "IDLE" ||
    normalized === "HOLD" ||
    normalized === "PAUSE";
  decisionText.textContent = value;
  if (decisionPhase) {
    decisionPhase.textContent = phase ? `PHASE ${phase}` : "PHASE --";
  }
  decisionBox.classList.toggle("decision-stop", isStop);
  decisionBox.classList.toggle("decision-go", !isStop);
}

function updateTarget(target) {
  if (!targetBox || !targetText) return;
  const label = target && String(target).trim().length ? String(target) : "";
  if (label) {
    targetText.textContent = label;
    targetBox.classList.remove("hidden");
  } else {
    targetText.textContent = "";
    targetBox.classList.add("hidden");
  }
}

function updateStatus(expReady, dacReady, hdopValue, satsValue) {
  const expOk = toBool(expReady) === true;
  const dacOk = toBool(dacReady) === true;
  setStatusIndicator(statusExp, expOk ? "status-ok" : "status-err", "EXP");
  setStatusIndicator(statusDac, dacOk ? "status-ok" : "status-err", "DAC");
  setHeaderLed(ledExp, expOk);
  setHeaderLed(ledDac, dacOk);

  const hdopNum =
    hdopValue === null || hdopValue === undefined
      ? NaN
      : Number(hdopValue);
  const hdopOk = Number.isFinite(hdopNum) && hdopNum < 2.0;
  const hdopText = Number.isFinite(hdopNum) ? hdopNum.toFixed(2) : "--";
  setStatusIndicator(
    statusHdop,
    hdopOk ? "status-ok" : "status-err",
    `HDOP ${hdopText}`
  );

  const satsNum =
    satsValue === null || satsValue === undefined
      ? NaN
      : Number(satsValue);
  const satsOk = Number.isFinite(satsNum) && satsNum > 6;
  const satsText = Number.isFinite(satsNum) ? satsNum.toFixed(0) : "--";
  setStatusIndicator(
    statusSats,
    satsOk ? "status-ok" : "status-err",
    `SATS ${satsText}`
  );
}

function updateMetrics(distanceValue, headingValue) {
  if (metricDistance) {
    const distNum =
      distanceValue === null || distanceValue === undefined
        ? NaN
        : Number(distanceValue);
    const distText = Number.isFinite(distNum) ? distNum.toFixed(1) : "--";
    metricDistance.textContent = `Distance: ${distText} m`;
  }
  if (metricHeading) {
    const headingNum =
      headingValue === null || headingValue === undefined
        ? NaN
        : Number(headingValue);
    const headingText = Number.isFinite(headingNum) ? headingNum.toFixed(0) : "--";
    metricHeading.textContent = `Heading: ${headingText} deg`;
  }
}

function renderTelemetry() {
  updateDecision(telemetry.decision, telemetry.phase);
  updateTarget(telemetry.target);
  updateStatus(
    telemetry.exp_ready,
    telemetry.dac_ready,
    telemetry.gps_hdop,
    telemetry.gps_sats
  );
  updateMetrics(telemetry.dist_m, telemetry.heading_deg);
}

function applyTelemetryMessage(msg) {
  if (!msg || typeof msg !== "object") return;
  if ("decision" in msg) telemetry.decision = msg.decision;
  if ("phase" in msg) telemetry.phase = msg.phase;
  if ("target_label" in msg) {
    telemetry.target = msg.target_label;
  } else if ("target" in msg) {
    telemetry.target = msg.target;
  }
  if ("dist_m" in msg) telemetry.dist_m = msg.dist_m;
  if ("heading_deg" in msg) {
    telemetry.heading_deg = msg.heading_deg;
  } else if ("heading" in msg) {
    telemetry.heading_deg = msg.heading;
  }
  if ("exp_ready" in msg) telemetry.exp_ready = msg.exp_ready;
  if ("dac_ready" in msg) telemetry.dac_ready = msg.dac_ready;
  if ("gps_hdop" in msg) telemetry.gps_hdop = msg.gps_hdop;
  if ("gps_sats" in msg) telemetry.gps_sats = msg.gps_sats;

  renderTelemetry();
}

function wsUrl(path) {
  const resolveHost = () => {
    try {
      const params = new URLSearchParams(window.location.search);
      const qsHost = params.get("wsHost") || params.get("wshost");
      if (qsHost && qsHost.trim().length) {
        localStorage.setItem(WS_HOST_OVERRIDE_KEY, qsHost.trim());
        return qsHost.trim();
      }
    } catch (e) {
      /* ignore */
    }
    try {
      const stored = localStorage.getItem(WS_HOST_OVERRIDE_KEY);
      if (stored && stored.trim().length) return stored.trim();
    } catch (e) {
      /* ignore */
    }
    if (window.location.host && window.location.host.trim().length) {
      return window.location.host;
    }
    return WS_FALLBACK_HOST;
  };

  const targetHost = resolveHost();
  const hostOnly = targetHost.split(":")[0];
  const isLocal =
    hostOnly === "localhost" ||
    hostOnly === "127.0.0.1" ||
    hostOnly.endsWith(".local") ||
    /^\d{1,3}(\.\d{1,3}){3}$/.test(hostOnly);
  const securePage = window.location.protocol === "https:";
  const scheme = securePage && !isLocal ? "wss:" : "ws:";
  return `${scheme}//${targetHost}${path}`;
}

function connectTelemetryWs() {
  if (
    telemetrySocket &&
    (telemetrySocket.readyState === WebSocket.OPEN ||
      telemetrySocket.readyState === WebSocket.CONNECTING)
  ) {
    return;
  }

  let ws;
  try {
    const url = wsUrl(TELEMETRY_PATH);
    console.info("[telemetry] connecting", url);
    ws = new WebSocket(url);
  } catch (err) {
    console.error("WebSocket init failed", err);
    setTimeout(connectTelemetryWs, TELEMETRY_RETRY_MS);
    return;
  }
  telemetrySocket = ws;
  ws.onopen = () => {
    console.info("[telemetry] connected");
    telemetryLastMessageAt = performance.now();
    telemetryStaleWarned = false;
  };
  ws.onmessage = (evt) => {
    telemetryLastMessageAt = performance.now();
    telemetryStaleWarned = false;
    try {
      const msg = JSON.parse(evt.data);
      applyTelemetryMessage(msg);
    } catch (e) {
      const preview =
        typeof evt.data === "string"
          ? evt.data.slice(0, 200)
          : evt.data instanceof ArrayBuffer
          ? `arraybuffer:${evt.data.byteLength}`
          : typeof evt.data;
      console.warn("[telemetry] parse error", e, preview);
    }
  };
  ws.onerror = (evt) => {
    console.warn("[telemetry] socket error, reconnecting...", evt);
    try {
      ws.close();
    } catch (e) {}
  };
  ws.onclose = (evt) => {
    telemetrySocket = null;
    console.warn("[telemetry] closed", evt.code, evt.reason, evt.wasClean);
    setTimeout(connectTelemetryWs, TELEMETRY_RETRY_MS);
  };
}

function updateNavButtonStyles() {
  if (btnGotoPost) {
    const active =
      navActiveTarget === "goto_post" || navLockedKey === "goto_post";
    btnGotoPost.classList.toggle("nav-active", active);
  }
  if (btnGotoH) {
    const active = navActiveTarget === "goto_h" || navLockedKey === "goto_h";
    btnGotoH.classList.toggle("nav-active", active);
  }
  Object.entries(navButtons).forEach(([key, btn]) => {
    if (!btn) return;
    const forceDisabled = btn.dataset.disabled === "true";
    const locked = navLockedKey !== null;
    const isActive = navLockedKey === key;
    btn.disabled = forceDisabled || (locked && !isActive);
    if (forceDisabled) {
      btn.setAttribute("aria-disabled", "true");
    }
    btn.setAttribute("aria-pressed", navActiveTarget === key ? "true" : "false");
  });

  if (btnStop) {
    const forceDisabled = btnStop.dataset.disabled === "true";
    btnStop.disabled = forceDisabled;
    btnStop.setAttribute("aria-pressed", "false");
    btnStop.classList.remove("nav-active");
  }
}

function syncGotoButtonsAvailability() {
  if (btnGotoPost) {
    const gotoPostEnabled = isSystemReady && !checkupInProgress;
    btnGotoPost.dataset.disabled = gotoPostEnabled ? "false" : "true";
  }
  if (btnGotoH) {
    const gotoHEnabled = isSystemReady && !checkupInProgress;
    btnGotoH.dataset.disabled = gotoHEnabled ? "false" : "true";
  }

  if (btnStop) {
    const stopEnabled = isSystemReady;
    btnStop.dataset.disabled = stopEnabled ? "false" : "true";
  }

  if (btnInitialCheckupRoutine) {
    btnInitialCheckupRoutine.disabled = checkupInProgress || isSystemReady;
    btnInitialCheckupRoutine.classList.toggle("hidden", isSystemReady);
  }

  updateNavButtonStyles();
}

function setNavTarget(target) {
  navActiveTarget = target;
  updateNavButtonStyles();
}

function clearNavTarget(options = {}) {
  navActiveTarget = null;
  if (options.skipNavLockClear) {
    updateNavButtonStyles();
  } else {
    clearNavLock();
  }
}

function lockNavButtons(activeKey) {
  navLockedKey = activeKey;
  updateNavButtonStyles();
}

function clearNavLock() {
  navLockedKey = null;
  updateNavButtonStyles();
}

function flashStopButton() {
  if (!btnStop) return;
  const now = Date.now();
  if (now - lastStopFlashAt < 350) return;
  lastStopFlashAt = now;
  btnStop.classList.add("stop-flash");
  if (stopFlashTimeout) {
    clearTimeout(stopFlashTimeout);
  }
  stopFlashTimeout = setTimeout(() => {
    btnStop.classList.remove("stop-flash");
    stopFlashTimeout = null;
  }, 1000);
}

function sendActuatorCommand(path) {
  return fetch(path, { method: "POST" }).catch(() => {});
}

function sendStopCommand() {
  flashStopButton();
  clearNavTarget({ skipNavLockClear: true });
  const navPromise = fetch("/nav/stop", { method: "POST" })
    .then((res) => {
      if (!res.ok) throw new Error("stop");
      return res.json();
    })
    .catch(() => {});
  const actuatorPromise = sendActuatorCommand("/actuator/stop");
  return Promise.allSettled([navPromise, actuatorPromise]);
}

connectTelemetryWs();
setInterval(() => {
  if (!telemetrySocket || telemetrySocket.readyState !== WebSocket.OPEN) return;
  if (!telemetryLastMessageAt) return;
  const idleMs = performance.now() - telemetryLastMessageAt;
  if (idleMs < TELEMETRY_STALE_MS || telemetryStaleWarned) return;
  telemetryStaleWarned = true;
  console.warn("[telemetry] stale", Math.round(idleMs), "ms since last message");
}, 1000);

if (btnGotoPost) {
  btnGotoPost.addEventListener("click", (event) => {
    event.preventDefault();
    event.stopPropagation();
    if (navLockedKey && navLockedKey !== "goto_post") return;
    lockNavButtons("goto_post");
    setNavTarget("goto_post");
    fetch("/nav/goto_post", { method: "POST" })
      .then((res) => {
        if (!res.ok) throw new Error("goto_post");
        return res.json();
      })
      .catch(() => {
        clearNavTarget();
      })
      .finally(() => {
        clearNavLock();
      });
  });
}
if (btnGotoH) {
  btnGotoH.addEventListener("click", (event) => {
    event.preventDefault();
    event.stopPropagation();
    if (navLockedKey && navLockedKey !== "goto_h") return;
    lockNavButtons("goto_h");
    setNavTarget("goto_h");
    fetch("/nav/goto_h", { method: "POST" })
      .then((res) => {
        if (!res.ok) throw new Error("goto_h");
        return res.json();
      })
      .catch(() => {
        clearNavTarget();
      })
      .finally(() => {
        clearNavLock();
      });
  });
}
if (btnStop) {
  btnStop.addEventListener("click", (event) => {
    event.preventDefault();
    event.stopPropagation();
    Promise.resolve(sendStopCommand()).finally(() => {
      clearNavTarget();
      clearNavLock();
    });
  });
}
if (btnInitialCheckupRoutine) {
  btnInitialCheckupRoutine.addEventListener("click", (event) => {
    event.preventDefault();
    event.stopPropagation();
    if (checkupInProgress || isSystemReady) return;
    checkupInProgress = true;
    checkupStartedAtMs = Date.now();
    syncGotoButtonsAvailability();
    if (checkupReadyTimeout) {
      clearTimeout(checkupReadyTimeout);
      checkupReadyTimeout = null;
    }

    fetch("/actuator/initial_checkup", { method: "POST" })
      .then(async (res) => {
        let payload = null;
        try {
          payload = await res.json();
        } catch (e) {
          payload = null;
        }
        return { ok: res.ok, payload };
      })
      .then(({ ok, payload }) => {
        const success = ok && payload && payload.queued === true;
        if (success) {
          const minReadyAt = checkupStartedAtMs + CHECKUP_MIN_READY_DELAY_MS;
          const bypass = payload && payload.bypass === true;
          const delayMs = bypass ? 0 : Math.max(0, minReadyAt - Date.now());
          checkupReadyTimeout = setTimeout(() => {
            checkupReadyTimeout = null;
            checkupInProgress = false;
            isSystemReady = true;
            syncGotoButtonsAvailability();
          }, delayMs);
          return;
        }
        checkupInProgress = false;
        syncGotoButtonsAvailability();
      })
      .catch(() => {
        checkupInProgress = false;
        syncGotoButtonsAvailability();
      });
  });
}

syncGotoButtonsAvailability();
renderTelemetry();
