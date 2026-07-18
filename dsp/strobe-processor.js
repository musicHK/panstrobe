// packages/dsp-core/src/math.ts
var TWO_PI = Math.PI * 2;
function ratioToCents(ratio) {
  return 1200 * Math.log2(ratio);
}
function wrapPhase(phi) {
  let p = phi % TWO_PI;
  if (p > Math.PI) p -= TWO_PI;
  else if (p <= -Math.PI) p += TWO_PI;
  return p;
}
function dbToLin(db) {
  return Math.pow(10, db / 20);
}
function linToDb(lin) {
  return 20 * Math.log10(lin);
}

// packages/dsp-core/src/filters.ts
var LpfCascade = class {
  constructor(sampleRate2, cutoffHz, numStages = 3) {
    this.sampleRate = sampleRate2;
    this.numStages = numStages;
    this.stages = new Float64Array(numStages);
    this.setCutoff(cutoffHz);
  }
  stages;
  a = 0;
  cutoffHz = 0;
  setCutoff(cutoffHz) {
    this.cutoffHz = cutoffHz;
    this.a = 1 - Math.exp(-TWO_PI * cutoffHz / this.sampleRate);
  }
  reset(value = 0) {
    this.stages.fill(value);
  }
  processSample(x) {
    const s = this.stages;
    const a = this.a;
    let v = x;
    for (let k = 0; k < s.length; k++) {
      s[k] += a * (v - s[k]);
      v = s[k];
    }
    return v;
  }
  /** Last output without advancing state. */
  get value() {
    return this.stages[this.stages.length - 1];
  }
};

// packages/dsp-core/src/band-demodulator.ts
function defaultLpfCutoffHz(freqHz) {
  return Math.min(25, Math.max(8, freqHz / 25));
}
var BandDemodulator = class {
  sampleRate;
  freqHz = 0;
  ncoPhase = 0;
  phaseInc = 0;
  iLpf;
  qLpf;
  iBar = 0;
  qBar = 0;
  samplesProcessed = 0;
  lastReadSample = 0;
  lastPhase = 0;
  hasLastPhase = false;
  // Outputs, updated by readFrame():
  /** Baseband vector angle atan2(Q̄, Ī), radians in (-π, π]. */
  phase = 0;
  /** Phase rate since previous readFrame, rad/s. Positive = sharp. */
  dPhiDt = 0;
  /** Baseband vector magnitude √(Ī²+Q̄²); ≈ half the partial's amplitude. */
  amplitude = 0;
  /** Frequency deviation from target, Hz (dPhiDt / 2π). */
  deltaFHz = 0;
  /** Frequency deviation from target, cents. */
  deltaCents = 0;
  /**
   * Unambiguous |Δf| for the last read interval: 1/(2·dt). A partial farther
   * off than this folds back inside the range and reads as a confident wrong
   * value, so consumers must cross-check against the FFT coarse deviation
   * (isFineReadingTrusted in protocol.ts) before trusting deltaCents.
   */
  aliasLimitHz = Infinity;
  constructor(opts) {
    this.sampleRate = opts.sampleRate;
    const stages = opts.lpfStages ?? 3;
    this.iLpf = new LpfCascade(opts.sampleRate, 15, stages);
    this.qLpf = new LpfCascade(opts.sampleRate, 15, stages);
  }
  setTarget(freqHz, opts) {
    this.freqHz = freqHz;
    this.phaseInc = TWO_PI * freqHz / this.sampleRate;
    const cutoff = opts?.lpfCutoffHz ?? defaultLpfCutoffHz(freqHz);
    this.iLpf.setCutoff(cutoff);
    this.qLpf.setCutoff(cutoff);
    if (opts?.resetFilters) {
      this.iLpf.reset();
      this.qLpf.reset();
      this.iBar = 0;
      this.qBar = 0;
      this.hasLastPhase = false;
      this.phase = 0;
      this.dPhiDt = 0;
      this.amplitude = 0;
      this.deltaFHz = 0;
      this.deltaCents = 0;
    }
  }
  setLpfCutoff(cutoffHz) {
    this.iLpf.setCutoff(cutoffHz);
    this.qLpf.setCutoff(cutoffHz);
  }
  get lpfCutoffHz() {
    return this.iLpf.cutoffHz;
  }
  processBlock(input) {
    let ph = this.ncoPhase;
    const inc = this.phaseInc;
    const iLpf = this.iLpf;
    const qLpf = this.qLpf;
    let iOut = this.iBar;
    let qOut = this.qBar;
    for (let n = 0; n < input.length; n++) {
      const x = input[n];
      iOut = iLpf.processSample(x * Math.cos(ph));
      qOut = qLpf.processSample(x * -Math.sin(ph));
      ph += inc;
      if (ph >= TWO_PI) ph -= TWO_PI;
    }
    this.ncoPhase = ph;
    this.iBar = iOut;
    this.qBar = qOut;
    this.samplesProcessed += input.length;
  }
  /**
   * Re-baseline phase tracking without touching the public readouts. Call
   * after a read gap (attack-skip, hold, retarget) so the accumulated phase
   * delta across the gap doesn't alias into a bogus dPhiDt on the next
   * readFrame.
   */
  primeFrame() {
    this.lastPhase = Math.atan2(this.qBar, this.iBar);
    this.lastReadSample = this.samplesProcessed;
    this.hasLastPhase = true;
  }
  /**
   * Update the public readouts from current filter state. Call at message
   * rate (~60–100 Hz), not per block: dPhiDt unwraps the phase delta since
   * the previous call by shortest path, so reads must be frequent enough
   * that |Δf| < readRate/2 stays unambiguous (beyond that the LPF has
   * suppressed the band anyway and the FFT coarse readout takes over).
   */
  readFrame() {
    const nowSample = this.samplesProcessed;
    const phi = Math.atan2(this.qBar, this.iBar);
    this.amplitude = Math.hypot(this.iBar, this.qBar);
    if (this.hasLastPhase && nowSample > this.lastReadSample) {
      const dtSec = (nowSample - this.lastReadSample) / this.sampleRate;
      this.dPhiDt = wrapPhase(phi - this.lastPhase) / dtSec;
      this.aliasLimitHz = 1 / (2 * dtSec);
    } else {
      this.dPhiDt = 0;
      this.aliasLimitHz = Infinity;
    }
    this.phase = phi;
    this.lastPhase = phi;
    this.lastReadSample = nowSample;
    this.hasLastPhase = true;
    this.deltaFHz = this.dPhiDt / TWO_PI;
    const ratio = 1 + this.deltaFHz / this.freqHz;
    this.deltaCents = this.freqHz > 0 && ratio > 0 ? ratioToCents(ratio) : 0;
  }
};

// packages/dsp-core/src/types.ts
var DEFAULT_ENGINE_PARAMS = {
  attackSkipMs: 120,
  holdEnabled: true,
  holdReleaseDb: -40,
  stableHoldEnabled: false,
  stableWindowMs: 500,
  stableSpreadCents: 2.5
};

// packages/dsp-core/src/protocol.ts
var SNAP_SEQ = 0;
var SNAP_TIME_SEC = 1;
var SNAP_INPUT_RMS = 2;
var SNAP_INPUT_PEAK = 3;
var SNAP_FLAGS = 4;
var SNAP_STRIKE_COUNT = 5;
var SNAP_BAND_COUNT = 6;
var SNAPSHOT_HEADER = 8;
var SNAPSHOT_BAND_STRIDE = 8;
var BAND_ID = 0;
var BAND_PHASE = 1;
var BAND_DPHIDT = 2;
var BAND_AMPLITUDE = 3;
var BAND_DELTA_CENTS = 4;
var BAND_DELTA_HZ = 5;
var BAND_FLAGS = 6;
var SNAP_FLAG_CLIPPING = 1;
var SNAP_FLAG_FROZEN = 2;
var SNAP_FLAG_HOLD = 4;
var SNAP_FLAG_ATTACK_SKIP = 8;
var BAND_FLAG_ACTIVE = 1;
var BAND_FLAG_GATED = 2;
var BAND_FLAG_HELD = 4;
function snapshotLength(maxBands) {
  return SNAPSHOT_HEADER + maxBands * SNAPSHOT_BAND_STRIDE;
}
var AUDIO_CHUNK_SIZE = 2048;

// packages/dsp-core/src/strike-detector.ts
var StrikeDetector = class {
  constructor(sampleRate2, opts) {
    this.sampleRate = sampleRate2;
    const attackMs = opts?.attackMs ?? 1;
    const releaseMs = opts?.releaseMs ?? 50;
    const slowMs = opts?.slowMs ?? 500;
    const slowDownMs = opts?.slowDownMs ?? 100;
    const triggerRatioDb = opts?.triggerRatioDb ?? 12;
    this.aFast = 1 - Math.exp(-1e3 / (attackMs * sampleRate2));
    this.rFast = 1 - Math.exp(-1e3 / (releaseMs * sampleRate2));
    this.aSlow = 1 - Math.exp(-1e3 / (slowMs * sampleRate2));
    this.dSlow = 1 - Math.exp(-1e3 / (slowDownMs * sampleRate2));
    this.triggerRatioLin = Math.pow(10, triggerRatioDb / 20);
    this.rearmRatioLin = Math.pow(10, triggerRatioDb / 40);
    this.floorLin = Math.pow(10, (opts?.floorDb ?? -50) / 20);
    this.refractorySamples = Math.round(
      (opts?.refractoryMs ?? 100) / 1e3 * sampleRate2
    );
    this.samplesSinceStrike = this.refractorySamples;
    this.slow = this.floorLin * 10;
  }
  aFast;
  rFast;
  aSlow;
  dSlow;
  triggerRatioLin;
  rearmRatioLin;
  floorLin;
  refractorySamples;
  fast = 0;
  slow;
  armed = true;
  samplesSinceStrike;
  /** Fast envelope level, linear — the "current loudness" other logic reads. */
  get fastLevel() {
    return this.fast;
  }
  /** Slow tracker in dBFS — a serviceable ambient-level estimate. */
  get noiseFloorDb() {
    return this.slow > 0 ? linToDb(this.slow) : -120;
  }
  /**
   * Returns the first strike detected within this block, or null. Envelope
   * state advances across the whole block regardless.
   */
  processBlock(input, blockStartSample) {
    let event = null;
    for (let n = 0; n < input.length; n++) {
      const x = Math.abs(input[n]);
      this.fast += (x > this.fast ? this.aFast : this.rFast) * (x - this.fast);
      this.slow += (this.fast > this.slow ? this.aSlow : this.dSlow) * (this.fast - this.slow);
      this.samplesSinceStrike++;
      const ratioHigh = this.fast > this.slow * this.triggerRatioLin;
      if (!this.armed) {
        if (this.fast < this.slow * this.rearmRatioLin) this.armed = true;
      } else if (event === null && ratioHigh && this.fast > this.floorLin && this.samplesSinceStrike >= this.refractorySamples) {
        event = { atSample: blockStartSample + n, peakLevel: this.fast };
        this.samplesSinceStrike = 0;
        this.armed = false;
      }
    }
    if (event) {
      let peak = event.peakLevel;
      for (let n = 0; n < input.length; n++) {
        const a = Math.abs(input[n]);
        if (a > peak) peak = a;
      }
      event.peakLevel = peak;
    }
    return event;
  }
};

// packages/dsp-core/src/strobe-engine.ts
var VALID_AMPLITUDE = 1e-4;
var ACTIVE_AMPLITUDE = 1e-3;
var STABLE_DECAY_RATIO = 0.7;
var STABLE_MIN_COVERAGE = 0.6;
var StrobeEngine = class {
  constructor(sampleRate2, maxBands = 8) {
    this.sampleRate = sampleRate2;
    this.maxBands = maxBands;
    this.bands = Array.from(
      { length: maxBands },
      () => new BandDemodulator({ sampleRate: sampleRate2 })
    );
    this.targets = new Array(maxBands).fill(null);
    this.detector = new StrikeDetector(sampleRate2);
    this.stableMin = new Float64Array(maxBands);
    this.stableMax = new Float64Array(maxBands);
    this.stableCount = new Int32Array(maxBands);
  }
  bands;
  targets;
  bandCount = 0;
  detector;
  params = { ...DEFAULT_ENGINE_PARAMS };
  state = "idle";
  frozen = false;
  strikeCount = 0;
  strikePeak = 0;
  skipUntilSample = 0;
  needsPrime = false;
  samplesProcessed = 0;
  lastReadSample = -1;
  blockRms = 0;
  blockPeak = 0;
  seq = 0;
  // Stable-hold window accumulators (fixed-size, allocation-free hot path).
  stableStartSample = -1;
  stableReads = 0;
  stableMin;
  stableMax;
  stableCount;
  onStrike;
  get currentState() {
    return this.state;
  }
  get currentBandCount() {
    return this.bandCount;
  }
  /** Full band reconfiguration: sets count/ids and resets all filter state. */
  configureBands(targets) {
    if (targets.length > this.maxBands) {
      throw new RangeError(
        `StrobeEngine: ${targets.length} bands exceeds maxBands=${this.maxBands}`
      );
    }
    this.bandCount = targets.length;
    for (let k = 0; k < this.maxBands; k++) {
      const t = targets[k] ?? null;
      this.targets[k] = t;
      if (t) {
        this.bands[k].setTarget(t.freqHz, {
          resetFilters: true,
          lpfCutoffHz: t.lpfCutoffHz
        });
      }
    }
    this.needsPrime = true;
    this.stableStartSample = -1;
    this.releaseHold();
  }
  /**
   * Update frequencies of the existing band set (matched by id; unmatched
   * targets are ignored). resetFilters=true on note changes kills the stale
   * baseband vector so the old note doesn't read as a deviation of the new.
   */
  retarget(targets, resetFilters) {
    for (const t of targets) {
      for (let k = 0; k < this.bandCount; k++) {
        if (this.targets[k]?.id === t.id) {
          this.targets[k] = t;
          this.bands[k].setTarget(t.freqHz, {
            resetFilters,
            lpfCutoffHz: t.lpfCutoffHz
          });
          break;
        }
      }
    }
    if (resetFilters) {
      this.needsPrime = true;
      this.stableStartSample = -1;
      this.releaseHold();
    }
  }
  setParams(p) {
    this.params = { ...this.params, ...p };
    if (this.params.holdEnabled === false && this.state === "hold") {
      this.releaseHold();
      this.needsPrime = true;
    }
  }
  setFrozen(on) {
    if (this.frozen && !on) {
      this.needsPrime = true;
      this.stableStartSample = -1;
    }
    this.frozen = on;
  }
  /**
   * Hold freezes readouts on a decayed note; a band-set change or disabling
   * hold must resume live readouts — the frozen values belong to the OLD
   * targets and must not be re-attributed to new ones. strikePeak resets so
   * hold can't immediately re-engage against a stale peak.
   */
  releaseHold() {
    if (this.state === "hold") {
      this.state = "idle";
      this.strikePeak = 0;
    }
  }
  processBlock(input) {
    let sumSq = 0;
    let peak = 0;
    for (let n = 0; n < input.length; n++) {
      const x = input[n];
      sumSq += x * x;
      const a = Math.abs(x);
      if (a > peak) peak = a;
    }
    this.blockRms = Math.sqrt(sumSq / input.length);
    this.blockPeak = peak;
    const strike = this.detector.processBlock(input, this.samplesProcessed);
    if (strike) {
      this.strikeCount++;
      this.strikePeak = strike.peakLevel;
      this.state = "attackSkip";
      this.skipUntilSample = strike.atSample + Math.round(this.params.attackSkipMs / 1e3 * this.sampleRate);
      this.stableStartSample = -1;
      this.onStrike?.(strike);
    }
    for (let k = 0; k < this.bandCount; k++) {
      this.bands[k].processBlock(input);
    }
    this.samplesProcessed += input.length;
    if (this.state === "attackSkip" && this.blockPeak > this.strikePeak) {
      this.strikePeak = this.blockPeak;
    }
    if (this.state === "attackSkip" && this.samplesProcessed >= this.skipUntilSample) {
      this.state = "live";
      this.needsPrime = true;
    }
    if (this.state === "live" && this.params.holdEnabled && this.strikePeak > 0 && this.detector.fastLevel < this.strikePeak * dbToLin(this.params.holdReleaseDb)) {
      this.state = "hold";
    }
  }
  /**
   * Advance band readouts per the state machine. Call at message rate before
   * fillSnapshot. Gated (attack-skip), held, and frozen states intentionally
   * leave the previous readings in place — the strobe freezes rather than
   * showing the glide or noise.
   */
  readFrames() {
    const updating = !this.frozen && (this.state === "live" || this.state === "idle");
    if (!updating) return;
    const gap = this.samplesProcessed - this.lastReadSample;
    if (this.lastReadSample >= 0 && gap > 4 * 512) this.needsPrime = true;
    this.lastReadSample = this.samplesProcessed;
    if (this.needsPrime) {
      for (let k = 0; k < this.bandCount; k++) this.bands[k].primeFrame();
      this.needsPrime = false;
      return;
    }
    for (let k = 0; k < this.bandCount; k++) this.bands[k].readFrame();
    if (this.params.stableHoldEnabled && this.params.holdEnabled && this.state === "live") {
      this.trackStability();
    }
  }
  /**
   * Auto-capture ("stable hold"): during post-strike live reading, watch
   * consecutive windows of stableWindowMs. If every ringing band's deltaCents
   * stays within stableSpreadCents while the level clearly decays below the
   * strike peak, the readings ARE the note — enter hold now instead of
   * waiting for the full decay, so short notes stay readable. A wobbling
   * (beating) band exceeds the spread and keeps its live readout; a
   * sustained tone never decays and never captures. Next strike releases.
   */
  trackStability() {
    if (this.stableStartSample < 0) {
      this.stableStartSample = this.samplesProcessed;
      this.stableReads = 0;
      for (let k = 0; k < this.bandCount; k++) {
        this.stableMin[k] = Infinity;
        this.stableMax[k] = -Infinity;
        this.stableCount[k] = 0;
      }
      return;
    }
    this.stableReads++;
    for (let k = 0; k < this.bandCount; k++) {
      const d = this.bands[k];
      const c = d.deltaCents;
      if (d.amplitude >= ACTIVE_AMPLITUDE && Number.isFinite(c)) {
        if (c < this.stableMin[k]) this.stableMin[k] = c;
        if (c > this.stableMax[k]) this.stableMax[k] = c;
        this.stableCount[k]++;
      }
    }
    const windowSamples = Math.round(
      this.params.stableWindowMs / 1e3 * this.sampleRate
    );
    if (this.samplesProcessed - this.stableStartSample < windowSamples) return;
    const decayed = this.strikePeak > 0 && this.detector.fastLevel < this.strikePeak * STABLE_DECAY_RATIO;
    const minCount = Math.max(
      4,
      Math.ceil(this.stableReads * STABLE_MIN_COVERAGE)
    );
    let anchored = 0;
    let stable = true;
    for (let k = 0; k < this.bandCount; k++) {
      if (this.stableCount[k] === 0) continue;
      if (this.bands[k].amplitude < ACTIVE_AMPLITUDE) continue;
      if (this.stableMax[k] - this.stableMin[k] > this.params.stableSpreadCents) {
        stable = false;
        break;
      }
      if (this.stableCount[k] >= minCount) anchored++;
    }
    if (decayed && stable && anchored > 0) {
      this.state = "hold";
    } else {
      this.stableStartSample = -1;
    }
  }
  fillSnapshot(buf) {
    buf[SNAP_SEQ] = this.seq++;
    buf[SNAP_TIME_SEC] = this.samplesProcessed / this.sampleRate;
    buf[SNAP_INPUT_RMS] = this.blockRms;
    buf[SNAP_INPUT_PEAK] = this.blockPeak;
    let flags = 0;
    if (this.blockPeak > 0.99) flags |= SNAP_FLAG_CLIPPING;
    if (this.frozen) flags |= SNAP_FLAG_FROZEN;
    if (this.state === "hold") flags |= SNAP_FLAG_HOLD;
    if (this.state === "attackSkip") flags |= SNAP_FLAG_ATTACK_SKIP;
    buf[SNAP_FLAGS] = flags;
    buf[SNAP_STRIKE_COUNT] = this.strikeCount;
    buf[SNAP_BAND_COUNT] = this.bandCount;
    buf[SNAPSHOT_HEADER - 1] = 0;
    for (let k = 0; k < this.bandCount; k++) {
      const demod = this.bands[k];
      const base = SNAPSHOT_HEADER + k * SNAPSHOT_BAND_STRIDE;
      buf[base + BAND_ID] = this.targets[k]?.id ?? -1;
      buf[base + BAND_PHASE] = demod.phase;
      buf[base + BAND_DPHIDT] = demod.dPhiDt;
      buf[base + BAND_AMPLITUDE] = demod.amplitude;
      buf[base + BAND_DELTA_CENTS] = demod.amplitude >= VALID_AMPLITUDE ? demod.deltaCents : NaN;
      buf[base + BAND_DELTA_HZ] = demod.deltaFHz;
      let bandFlags = 0;
      if (demod.amplitude >= ACTIVE_AMPLITUDE) bandFlags |= BAND_FLAG_ACTIVE;
      if (this.state === "attackSkip") bandFlags |= BAND_FLAG_GATED;
      if (this.state === "hold") bandFlags |= BAND_FLAG_HELD;
      buf[base + BAND_FLAGS] = bandFlags;
      buf[base + 7] = 0;
    }
  }
};

// packages/worklet/src/strobe-processor.ts
var MAX_BANDS = 64;
var SNAPSHOT_EVERY_SAMPLES = 512;
var SNAPSHOT_POOL = 12;
var AUDIO_POOL = 8;
var StrobeProcessor = class extends AudioWorkletProcessor {
  engine = new StrobeEngine(sampleRate, MAX_BANDS);
  snapPool = [];
  audioPool = [];
  /** Samples ingested since the last snapshot (any chunk size). */
  sinceSnap = 0;
  /** External-PCM mode (native capture): mic input quanta are ignored and
   *  audio arrives as transferred buffers over the port instead. */
  external = false;
  /** Raw-audio staging: accumulate arbitrary chunks for the analysis path. */
  stage = new Float32Array(AUDIO_CHUNK_SIZE);
  stageFill = 0;
  constructor() {
    super();
    const snapBytes = snapshotLength(MAX_BANDS) * 4;
    for (let i = 0; i < SNAPSHOT_POOL; i++) {
      this.snapPool.push(new ArrayBuffer(snapBytes));
    }
    for (let i = 0; i < AUDIO_POOL; i++) {
      this.audioPool.push(new ArrayBuffer(AUDIO_CHUNK_SIZE * 4));
    }
    this.hookEngine();
    this.port.onmessage = (event) => {
      const msg = event.data;
      switch (msg.type) {
        case "configure":
          this.engine.configureBands(msg.bands);
          break;
        case "retarget":
          this.engine.retarget(msg.bands, msg.resetFilters);
          break;
        case "freeze":
          this.engine.setFrozen(msg.on);
          break;
        case "params":
          this.engine.setParams(msg.params);
          break;
        case "external":
          this.external = msg.sampleRate > 0;
          this.engine = new StrobeEngine(
            this.external ? msg.sampleRate : sampleRate,
            MAX_BANDS
          );
          this.hookEngine();
          this.sinceSnap = 0;
          this.stageFill = 0;
          break;
        case "pcm":
          if (this.external) this.ingest(new Float32Array(msg.buffer));
          break;
        case "return":
          if (this.snapPool.length < SNAPSHOT_POOL) this.snapPool.push(msg.buffer);
          break;
        case "return-audio":
          if (this.audioPool.length < AUDIO_POOL) this.audioPool.push(msg.buffer);
          break;
      }
    };
    this.port.postMessage({ type: "ready", sampleRate });
  }
  hookEngine() {
    this.engine.onStrike = (e) => {
      this.port.postMessage({
        type: "strike",
        atSample: e.atSample,
        peakLevel: e.peakLevel
      });
    };
  }
  process(inputs) {
    const channel = inputs[0]?.[0];
    if (!this.external && channel && channel.length > 0) {
      this.ingest(channel);
    }
    return true;
  }
  /**
   * Run a mono chunk of ANY length through the engine, forward it to the
   * analysis path, and emit snapshots on the sample-count cadence. Shared
   * by the render-quantum path (128 samples) and external PCM (native
   * capture chunks, typically ~2048 but hardware-decided).
   *
   * The chunk is SLICED at the snapshot cadence: readFrames() computes
   * dφ/dt from the samples advanced since the previous read, so it must
   * observe the engine after EACH 512-sample step. One bulk processBlock
   * over a native chunk followed by back-to-back reads would make reads
   * 2..n zero-dt — dPhiDt and deltaCents pinned to 0, the median filter
   * swallowed by zeros, sub-cent accuracy silently destroyed
   * (adversarial-review P1). Slicing also keeps the read gap at exactly
   * 512 for any hardware buffer size, so the re-prime guard never trips.
   */
  ingest(channel) {
    let off = 0;
    while (off < channel.length) {
      const n = Math.min(
        SNAPSHOT_EVERY_SAMPLES - this.sinceSnap,
        channel.length - off
      );
      const slice = channel.subarray(off, off + n);
      this.engine.processBlock(slice);
      this.forwardAudio(slice);
      off += n;
      this.sinceSnap += n;
      if (this.sinceSnap >= SNAPSHOT_EVERY_SAMPLES) {
        this.sinceSnap = 0;
        const buffer = this.snapPool.pop();
        if (buffer) {
          this.engine.readFrames();
          this.engine.fillSnapshot(new Float32Array(buffer));
          this.port.postMessage(buffer, [buffer]);
        }
      }
    }
  }
  /** Stage raw audio for the analysis worker (any slice length). */
  forwardAudio(slice) {
    let off = 0;
    while (off < slice.length) {
      const n = Math.min(AUDIO_CHUNK_SIZE - this.stageFill, slice.length - off);
      this.stage.set(slice.subarray(off, off + n), this.stageFill);
      this.stageFill += n;
      off += n;
      if (this.stageFill >= AUDIO_CHUNK_SIZE) {
        this.stageFill = 0;
        const buffer = this.audioPool.pop();
        if (buffer) {
          new Float32Array(buffer).set(this.stage);
          this.port.postMessage({ type: "audio", buffer }, [buffer]);
        }
      }
    }
  }
};
registerProcessor("strobe-processor", StrobeProcessor);
//# sourceMappingURL=strobe-processor.js.map
