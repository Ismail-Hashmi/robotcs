#include <RobotisOp2GaitManager.hpp>
#include <RobotisOp2MotionManager.hpp>
#include <webots/Camera.hpp>
#include <webots/Gyro.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/Speaker.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace webots;
using namespace managers;

enum State { SEARCH, APPROACH, ALIGN, KICK };

class StableOp2 : public Robot {
public:
  StableOp2()
    : headYaw(nullptr), headPitch(nullptr), prevBallX(0), prev2BallX(0), prevBallTime(0.0), hadBall(false),
      previousState(SEARCH), lastKickTime(-100.0), prevOppCentroidX(0), prevOppTime(0.0), hadOpp(false) {
    timeStep = (int)getBasicTimeStep();

    camera = getCamera("Camera");
    if (!camera) {
      std::cerr << "FATAL: device 'Camera' not found on this robot.\n";
      exit(1);
    }
    camera->enable(timeStep);

    gyro = getGyro("Gyro");
    if (gyro)
      gyro->enable(timeStep);

    speaker = getSpeaker("Speaker");

    motion = new RobotisOp2MotionManager(this, "");
    gait = new RobotisOp2GaitManager(this, "config.ini");

    if (!motion->isCorrectlyInitialized()) {
      std::cerr << "FATAL: RobotisOp2MotionManager failed (motion_4096.bin?).\n";
      exit(1);
    }
    if (!gait->isCorrectlyInitialized()) {
      std::cerr << "FATAL: RobotisOp2GaitManager failed (config.ini?).\n";
      exit(1);
    }

    headYaw = getMotor("head_yaw");
    headPitch = getMotor("head_pitch");
    if (!headYaw)
      headYaw = getMotor("Neck");
    if (!headPitch)
      headPitch = getMotor("Head");

    neck = headYaw;
    head = headPitch;

    applyHeadPose();
  }

  void run() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);

    const bool motionOk = motion->isCorrectlyInitialized();
    const bool gaitOk = gait->isCorrectlyInitialized();
    std::cerr << "\n========== SOCCER StableOp2 (vision: ball, 2-post goal, mates, opp, lines) ==========\n"
              << "timeStep=" << timeStep << " ms\n"
              << "motionManager init=" << motionOk << " motionPlaying=" << motion->isMotionPlaying() << "\n"
              << "gaitManager init=" << gaitOk << "\n"
              << "head_yaw=" << (headYaw ? "ok" : "null") << " head_pitch=" << (headPitch ? "ok" : "null")
              << "\n================================================================================\n"
              << std::flush;

    enableOp2PositionSensors();
    for (int k = 0; k < 4; k++)
      step(timeStep);

    motion->playPage(1);
    while (motion->isMotionPlaying())
      step(timeStep);

    motion->playPage(9);
    while (motion->isMotionPlaying())
      step(timeStep);

    for (int i = 0; i < 6; i++)
      step(timeStep);

    gait->start();
    for (int i = 0; i < 14; i++) {
      double ramp = i / 14.0;
      gait->setXAmplitude(0.15 * ramp);
      gait->setAAmplitude(0.0);
      gait->step(timeStep);
      step(timeStep);
    }

    while (step(timeStep) != -1) {
      int objX = 0, objSize = 0;
      bool found = detectBall(objX, objSize);

      int width = camera->getWidth();
      int height = camera->getHeight();
      const double center = width * 0.5;
      const double kickXRef = (width <= 360) ? 160.0 : center;
      const double kickCenterTol = 20.0;

      applyHeadPose();

      double t = getTime();
      double ballSpeed = 0.0;
      double ballAccelX = 0.0;
      if (found && hadBall) {
        double dt = t - prevBallTime;
        if (dt > 1e-4) {
          ballSpeed = (objX - prevBallX) / dt;
          double dt2 = t - prevBallTime;
          if (dt2 > 1e-4 && prev2BallX != prevBallX)
            ballAccelX = ((objX - prevBallX) - (prevBallX - prev2BallX)) / (dt2 * dt2);
        }
      }
      if (found) {
        prev2BallX = prevBallX;
        prevBallX = objX;
        prevBallTime = t;
        hadBall = true;
      } else {
        hadBall = false;
      }

      int goalLeftX = width / 2;
      int goalRightX = width / 2;
      int goalMidX = width / 2;
      int yellowPx = 0;
      bool goalTwoPosts = false;
      bool goalSeen = detectGoalTwoPosts(goalLeftX, goalRightX, goalMidX, yellowPx, goalTwoPosts);

      int robotPixels = 0;
      int oppCentroidX = width / 2;
      detectOpponentBlue(robotPixels, oppCentroidX);

      double oppSpeed = 0.0;
      if (robotPixels > 0 && hadOpp) {
        double dtp = t - prevOppTime;
        if (dtp > 1e-4)
          oppSpeed = (oppCentroidX - prevOppCentroidX) / dtp;
      }
      if (robotPixels > 0) {
        prevOppCentroidX = oppCentroidX;
        prevOppTime = t;
        hadOpp = true;
      } else {
        hadOpp = false;
      }

      int matePixels = 0;
      int mateCentroidX = width / 2;
      detectTeammateRed(matePixels, mateCentroidX);

      int lineTotal = 0;
      int lineBoundary = 0;
      int lineCenterCircle = 0;
      int linePenalty = 0;
      classifyFieldLines(height, width, lineTotal, lineBoundary, lineCenterCircle, linePenalty);

      bool ballCenteredKick = found && fabs((double)objX - kickXRef) < kickCenterTol;
      bool goalAligned =
          goalSeen && fabs((double)goalMidX - center) < (double)kGoalCenterTolPx;

      bool canKickAgain = (t - lastKickTime) >= kKickCooldown;
      bool kickReady =
          found && (objSize > kKickMinBallPixels) && ballCenteredKick && canKickAgain;

      State state;
      if (!found)
        state = SEARCH;
      else if (robotPixels > kAvoidBlueThreshold && !(found && objSize >= kBallApproachIgnoreBlueMin))
        state = APPROACH;
      else if (found && goalSeen && !goalAligned && objSize >= kAlignMinBallPixels)
        state = ALIGN;
      else if (kickReady)
        state = KICK;
      else
        state = APPROACH;

      double xAmp = 0.0;
      double aAmp = 0.0;

      static int logStep = 0;
      logStep++;
      bool doLog = (logStep % kLogEveryNSteps == 0);

      if (state != KICK) {
        switch (state) {
          case SEARCH:
            xAmp = 0.0;
            aAmp = 0.3;
            break;

          case APPROACH: {
            double turn = ((double)objX - center) / std::max(1.0, center);
            turn = clamp(turn, -0.5, 0.5);
            const bool heavyBlue = robotPixels > kAvoidBlueThreshold;
            const bool ballChase = found && objSize >= kBallApproachIgnoreBlueMin;
            // Always yaw toward ball centroid; only reduce forward speed when blue is high and ball still small.
            if (heavyBlue && !ballChase) {
              xAmp = 0.18;
              aAmp = clamp(-turn * 1.15, -0.48, 0.48);
            } else {
              xAmp = 1.0;
              aAmp = -turn;
            }
            if (matePixels > kMateCrowdPixels && fabs((double)mateCentroidX - center) < (double)width * 0.22) {
              xAmp *= 0.72;
              aAmp += (mateCentroidX < (int)center ? 0.12 : -0.12);
              aAmp = clamp(aAmp, -0.45, 0.45);
            }
            break;
          }

          case ALIGN: {
            xAmp = 0.0;
            double gNorm = 2.0 * goalMidX / (double)width - 1.0;
            aAmp = clamp(-0.42 * gNorm, -0.48, 0.48);
            break;
          }

          default:
            break;
        }

        gait->setXAmplitude(xAmp);
        gait->setAAmplitude(aAmp);
        gait->step(timeStep);

        if (doLog) {
          std::cerr << std::fixed << std::setprecision(2)
                    << "[move] state=" << stateName(state) << " xAmp=" << xAmp << " aAmp=" << aAmp
                    << " ball=" << (found ? "yes" : "no") << " x=" << objX << " sz=" << objSize << "\n";
          std::cerr << std::fixed << std::setprecision(1)
                    << "[ball] vx=" << ballSpeed << " ax~=" << ballAccelX << " | [goal] mid=" << goalMidX
                    << " L=" << goalLeftX << " R=" << goalRightX << " yel=" << yellowPx
                    << " 2post=" << (goalTwoPosts ? 1 : 0) << "\n";
          std::cerr << std::fixed << std::setprecision(1)
                    << "[agents] bluePx=" << robotPixels << " oppX=" << oppCentroidX << " ovx=" << oppSpeed
                    << " | redPx=" << matePixels << " mateX=" << mateCentroidX << "\n";
          std::cerr << std::fixed << std::setprecision(0)
                    << "[lines] tot=" << lineTotal << " bound=" << lineBoundary << " circ=" << lineCenterCircle
                    << " pen=" << linePenalty << std::endl;
        }
      }

      if (state == KICK && previousState != KICK) {
        std::cerr << "[move] KICK gait->stop() sz=" << objSize << " |x-kickRef|<" << kickCenterTol
                  << " kickRef=" << kickXRef << "\n"
                  << std::flush;

        gait->stop();

        for (int i = 0; i < 4; i++)
          step(timeStep);

        motion->playPage(9);
        while (motion->isMotionPlaying())
          step(timeStep);

        // Yellow goal midpoint left of image centre → kick with page 12; right → 13 (swap if shots mirror wrong way).
        int kickPage = (goalMidX < (int)center) ? 12 : 13;
        std::cerr << "[move] KICK playPage(" << kickPage << ")\n" << std::flush;
        motion->playPage(kickPage);
        while (motion->isMotionPlaying())
          step(timeStep);

        motion->playPage(9);
        while (motion->isMotionPlaying())
          step(timeStep);

        gait->start();
        for (int i = 0; i < 8; i++) {
          double r = (i + 1) / 8.0;
          gait->setXAmplitude(0.12 * r);
          gait->setAAmplitude(0.0);
          gait->step(timeStep);
          step(timeStep);
        }

        lastKickTime = getTime();
        std::cerr << "[move] KICK complete, cooldown=" << kKickCooldown << "s\n" << std::flush;
      } else if (state == KICK) {
        gait->setXAmplitude(0.0);
        gait->setAAmplitude(0.0);
        gait->step(timeStep);
      }

      previousState = state;
    }
  }

private:
  int timeStep;
  Camera *camera;
  Gyro *gyro;
  Speaker *speaker;
  Motor *neck;
  Motor *head;
  Motor *headYaw;
  Motor *headPitch;

  RobotisOp2MotionManager *motion;
  RobotisOp2GaitManager *gait;

  int prevBallX;
  int prev2BallX;
  double prevBallTime;
  bool hadBall;
  State previousState;
  double lastKickTime;

  int prevOppCentroidX;
  double prevOppTime;
  bool hadOpp;

  static constexpr double kHeadPitchTargetRad = -0.35;

  void applyHeadPose() {
    if (headYaw)
      headYaw->setPosition(0.0);
    if (!headPitch)
      return;
    double p = kHeadPitchTargetRad;
    const double lo = headPitch->getMinPosition();
    const double hi = headPitch->getMaxPosition();
    if (std::isfinite(lo))
      p = std::max(p, lo);
    if (std::isfinite(hi))
      p = std::min(p, hi);
    headPitch->setPosition(p);
  }

  void enableOp2PositionSensors() {
    static const char *names[] = {"ShoulderR", "ShoulderL", "ArmUpperR", "ArmUpperL", "ArmLowerR", "ArmLowerL",
                                  "PelvYR",    "PelvYL",    "PelvR",     "PelvL",     "LegUpperR", "LegUpperL",
                                  "LegLowerR", "LegLowerL", "AnkleR",    "AnkleL",    "FootR",     "FootL",
                                  "Neck",      "Head"};
    for (const char *mn : names) {
      std::string sn = std::string(mn) + "S";
      PositionSensor *ps = getPositionSensor(sn);
      if (ps)
        ps->enable(timeStep);
    }
  }

  static constexpr int kGoalCenterTolPx = 44;
  static const int kAvoidBlueThreshold = 2600;
  // When the ball blob is at least this big, always steer toward the ball (do not use
  // fixed "avoidance" yaw that ignores ball bearing — that made the robot curve away from the ball).
  static const int kBallApproachIgnoreBlueMin = 350;
  static const int kMateCrowdPixels = 650;
  static const int kLogEveryNSteps = 30;
  static const int kVisionScanStep = 6;
  static const int kKickMinBallPixels = 9000;
  // Only enter ALIGN when the ball blob is this large; otherwise goal-in-view would
  // freeze forward gait (xAmp=0) while the ball is still small / far away.
  static const int kAlignMinBallPixels = 5200;
  static constexpr double kKickCooldown = 3.5;
  static const int kMinGapSplitPostsPx = 22;

  static int samplesInScan(int w, int h, int step) {
    int nx = (w + step - 1) / step;
    int ny = (h + step - 1) / step;
    return std::max(1, nx * ny);
  }

  static const char *stateName(State s) {
    switch (s) {
      case SEARCH:
        return "SEARCH";
      case APPROACH:
        return "APPROACH";
      case ALIGN:
        return "ALIGN";
      case KICK:
        return "KICK";
      default:
        return "?";
    }
  }

  double clamp(double v, double lo, double hi) {
    return std::max(lo, std::min(v, hi));
  }

  bool detectBall(int &xCenter, int &size) {
    const unsigned char *img = camera->getImage();
    int w = camera->getWidth();
    int h = camera->getHeight();
    const int st = kVisionScanStep;

    long sumX = 0;
    int count = 0;

    for (int x = 0; x < w; x += st) {
      for (int y = 0; y < h; y += st) {
        int r = Camera::imageGetRed(img, w, x, y);
        int gc = Camera::imageGetGreen(img, w, x, y);
        int b = Camera::imageGetBlue(img, w, x, y);

        bool grass = (gc > r + 22 && gc > b + 22 && r < 130);

        bool orange = (r > 150 && gc < 120 && b < 120);
        bool robocupDark = (r < 95 && gc < 95 && b < 95);
        bool robocupWhite = (!grass && r > 165 && gc > 165 && b > 165 && y > h / 6);

        if (orange || robocupDark || robocupWhite) {
          count++;
          sumX += x;
        }
      }
    }

    int sRef = samplesInScan(w, h, 3);
    int sCur = samplesInScan(w, h, st);
    int minCount = std::max(24, (90 * sCur) / std::max(1, sRef));

    if (count < minCount) {
      size = count;
      xCenter = w / 2;
      return false;
    }

    size = count;
    xCenter = (int)(sumX / count);
    return true;
  }

  // Yellow goal posts: 1D gap split → left / right post centroids + midpoint (bearing target).
  bool detectGoalTwoPosts(int &goalLeftX, int &goalRightX, int &goalMidX, int &yellowPx, bool &twoPosts) {
    const unsigned char *img = camera->getImage();
    int w = camera->getWidth();
    int h = camera->getHeight();
    const int st = kVisionScanStep;

    std::vector<int> xs;
    xs.reserve(512);

    for (int x = 0; x < w; x += st) {
      for (int y = 0; y < h; y += st) {
        int r = Camera::imageGetRed(img, w, x, y);
        int g = Camera::imageGetGreen(img, w, x, y);
        int b = Camera::imageGetBlue(img, w, x, y);

        if (r > 175 && g > 175 && b < 130) {
          xs.push_back(x);
        }
      }
    }

    yellowPx = (int)xs.size();
    int sRef = samplesInScan(w, h, 3);
    int sCur = samplesInScan(w, h, st);
    int minGoal = std::max(20, (100 * sCur) / std::max(1, sRef));
    if (yellowPx < minGoal)
      return false;

    std::sort(xs.begin(), xs.end());

    int bestGap = 0;
    size_t splitAfter = 0;
    for (size_t i = 0; i + 1 < xs.size(); ++i) {
      int g = xs[i + 1] - xs[i];
      if (g > bestGap) {
        bestGap = g;
        splitAfter = i;
      }
    }

    twoPosts = (bestGap >= kMinGapSplitPostsPx && xs.size() >= 30);

    if (!twoPosts) {
      long s = 0;
      for (int x : xs)
        s += x;
      goalMidX = (int)(s / (long)xs.size());
      goalLeftX = goalRightX = goalMidX;
      return true;
    }

    int splitX = (xs[splitAfter] + xs[splitAfter + 1]) / 2;
    long sumL = 0, sumR = 0;
    int nL = 0, nR = 0;
    for (int x : xs) {
      if (x < splitX) {
        sumL += x;
        nL++;
      } else {
        sumR += x;
        nR++;
      }
    }
    if (nL < 1 || nR < 1) {
      long s = 0;
      for (int x : xs)
        s += x;
      goalMidX = (int)(s / (long)xs.size());
      goalLeftX = goalRightX = goalMidX;
      twoPosts = false;
      return true;
    }
    goalLeftX = (int)(sumL / nL);
    goalRightX = (int)(sumR / nR);
    if (goalLeftX > goalRightX)
      std::swap(goalLeftX, goalRightX);
    goalMidX = (goalLeftX + goalRightX) / 2;
    return true;
  }

  void detectOpponentBlue(int &blueCount, int &centroidX) {
    const unsigned char *img = camera->getImage();
    int w = camera->getWidth();
    int h = camera->getHeight();
    int y0 = h / 2;
    const int st = kVisionScanStep;

    blueCount = 0;
    long sumX = 0;
    for (int x = 0; x < w; x += st) {
      for (int y = y0; y < h; y += st) {
        int r = Camera::imageGetRed(img, w, x, y);
        int g = Camera::imageGetGreen(img, w, x, y);
        int b = Camera::imageGetBlue(img, w, x, y);
        if (b > 140 && r < 110 && g < 110) {
          blueCount++;
          sumX += x;
        }
      }
    }
    centroidX = (blueCount > 0) ? (int)(sumX / blueCount) : w / 2;
  }

  // Red jersey (teammate): exclude orange ball mask and very dark ball panels.
  void detectTeammateRed(int &redCount, int &centroidX) {
    const unsigned char *img = camera->getImage();
    int w = camera->getWidth();
    int h = camera->getHeight();
    int y0 = h / 3;
    const int st = kVisionScanStep;

    redCount = 0;
    long sumX = 0;
    for (int x = 0; x < w; x += st) {
      for (int y = y0; y < h; y += st) {
        int r = Camera::imageGetRed(img, w, x, y);
        int gc = Camera::imageGetGreen(img, w, x, y);
        int b = Camera::imageGetBlue(img, w, x, y);

        bool orangeBall = (r > 150 && gc < 120 && b < 120);
        bool darkBall = (r < 95 && gc < 95 && b < 95);
        if (orangeBall || darkBall)
          continue;

        bool redJersey =
            (r > 88 && r > gc + 22 && r > b + 22 && gc < 135 && b < 135 && (r + gc + b) > 140);
        if (redJersey) {
          redCount++;
          sumX += x;
        }
      }
    }
    centroidX = (redCount > 0) ? (int)(sumX / redCount) : w / 2;
  }

  // ROI "Hough-lite": classify line-like whites by image region (boundary vs centre circle vs penalty hints).
  void classifyFieldLines(int h, int w, int &whiteTotal, int &scoreBoundary, int &scoreCenterCircle, int &scorePenalty) {
    const unsigned char *img = camera->getImage();
    const int st = kVisionScanStep;

    whiteTotal = 0;
    scoreBoundary = 0;
    scoreCenterCircle = 0;
    scorePenalty = 0;

    const int yB0 = (h * 3) / 4;
    const int yC0 = h / 4;
    const int yC1 = (h * 3) / 4;
    const int xBand = std::max(8, w / 5);

    for (int x = 0; x < w; x += st) {
      for (int y = 0; y < h; y += st) {
        int r = Camera::imageGetRed(img, w, x, y);
        int g = Camera::imageGetGreen(img, w, x, y);
        int b = Camera::imageGetBlue(img, w, x, y);
        if (r <= 195 || g <= 195 || b <= 195)
          continue;

        whiteTotal++;
        if (y >= yB0)
          scoreBoundary++;
        if (y >= yC0 && y <= yC1 && std::abs(x - w / 2) < (w / 2 - xBand))
          scoreCenterCircle++;
        if (y > h / 2 && (x < xBand || x > w - xBand))
          scorePenalty++;
      }
    }
  }
};

int main() {
  StableOp2 robot;
  robot.run();
  return 0;
}
