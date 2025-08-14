#pragma once
#include "SimplePID.h"

class SimplePID_Auto {
public:
  enum class Mode { PID, AUTOTUNE, HOLD };

  // コンストラクタで PID インスタンスを受け取る
  SimplePID_Auto(SimplePID& pid,
                 float relayAmp          = 150.0f,   // ±出力 [PWM, mA ...]
                 float hysteresis        = 0.0f,      // 0 で十分
                 uint8_t cyclesToRecord  = 6)
    : pid_(pid),
      H_(relayAmp),
      hyst_(hysteresis),
      N_(cyclesToRecord) {}

  /* 呼び出しは loop() から 1 kHz 程度で OK */
  float update(float input, float setpoint) {
    switch (mode_) {
      case Mode::PID:
        return pid_.compute(input, setpoint);

      case Mode::AUTOTUNE:
        return updateRelay(input, setpoint);

      case Mode::HOLD:
      default:
        return 0.0f; // 出力せずに計算待ち
    }
  }

  /* オートチューニング開始 */
  void startAutotune(float setpoint) {
    atSetpoint_ = setpoint;
    samples_.clear();
    sign_ = 1;
    prevErr_ = 0;
    lastCross_ = micros();
    mode_ = Mode::AUTOTUNE;
  }

  /* PID 係数を読み出し */
  bool isFinished() const { return finished_; }
  void getGains(float& kp, float& ki, float& kd) const {
    kp = newKp_;  ki = newKi_;  kd = newKd_;
  }

private:
  // ---------- リレー出力 & 計測 ----------
  float updateRelay(float input, float /*setpoint*/) {
    float err = atSetpoint_ - input;

    /* 零交差を検出 */
    if ((err > 0 && prevErr_ <= 0) || (err < 0 && prevErr_ >= 0)) {
      uint32_t now = micros();
      uint32_t halfTd = now - lastCross_;
      lastCross_ = now;

      /* 振幅 a = 平均(|err|) を近似: リレー法では出力の方で OK */
      float a = fabsf(lastOut_); // = H_
      samples_.push_back({float(halfTd) * 1e-6f, a});
      if (samples_.size() >= N_ * 2) computeGains(); // N 周期

      // 収集完了まで HOLD にしてもいい
    }

    /* ヒステリシスでチャタリング防止（なくても可） */
    if (err >  hyst_) sign_ = +1;
    if (err < -hyst_) sign_ = -1;

    lastOut_ = sign_ * H_;
    prevErr_ = err;
    return lastOut_;
  }

  // ---------- Ku, Pu からゲイン算出 ----------
  void computeGains() {
    // 外れ値除去：IQR フィルタ
    std::vector<float> Pu, a;
    for (auto& s : samples_) { Pu.push_back(2.0f * s.half); a.push_back(s.amp); }
    auto clean = [](std::vector<float>& v) {
      std::sort(v.begin(), v.end());
      size_t q1 = v.size() / 4, q3 = 3 * v.size() / 4;
      float iqr = v[q3] - v[q1];
      float lo = v[q1] - 1.5f * iqr, hi = v[q3] + 1.5f * iqr;
      std::vector<float> r;
      for (float x : v) if (x >= lo && x <= hi) r.push_back(x);
      v.swap(r);
    };
    clean(Pu); clean(a);

    float Pu_avg = std::accumulate(Pu.begin(), Pu.end(), 0.0f) / Pu.size();
    float ampAvg = std::accumulate(a.begin(),  a.end(),  0.0f) / a.size();

    // Ku = 4H / (π a)  (Åström‑Hägglund 原式)
    float Ku = 4.0f * H_ / (M_PI * ampAvg);

    /* ---- Tyreus‑Luyben 推奨値 ----
       Kp = Ku / 2.2
       Ti = 2.2 * Pu
       Td = 0.168 * Pu
    */
    newKp_ = Ku / 2.2f;
    float Ti = 2.2f * Pu_avg;
    float Td = 0.168f * Pu_avg;
    newKi_ = newKp_ / Ti;
    newKd_ = newKp_ * Td;

    /* PID へ反映 */
    pid_ = SimplePID(newKp_, newKi_, newKd_,
                     pidOutMin_, pidOutMax_, pidSampleMs_);

    finished_ = true;
    mode_ = Mode::PID;
  }

  // ---------- 内部状態 ----------
  SimplePID& pid_;
  Mode mode_{Mode::PID};

  // --- リレー法パラメータ
  float H_, hyst_;
  uint8_t N_;
  struct Sample { float half; float amp; };
  std::vector<Sample> samples_;

  // --- 計測用
  int8_t   sign_{+1};
  float    prevErr_{0}, lastOut_{0};
  uint32_t lastCross_{0};
  float    atSetpoint_{0};

  // --- ゲイン
  float newKp_{0}, newKi_{0}, newKd_{0};
  bool  finished_{false};

  // --- PID コンストラクタ用の元値保持
  float pidOutMin_{-255}, pidOutMax_{255};
  uint32_t pidSampleMs_{1};
};
