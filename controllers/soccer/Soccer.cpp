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

using namespace webots;
using namespace managers;

enum State { SEARCH, APPROACH, ALIGN, KICK };

class StableOp2 : public Robot {
public:
  StableOp2()
    : headYaw(nullptr), headPitch(nullptr), prevBallX(0), prevBallTime(0.0), hadBall(false),
      previousState(SEARCH), lastKickTime(-100.0) {
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

    if (headYaw)
      headYaw->setPosition(0.0);
    if (headPitch)
      headPitch->setPosition(-0.4);
  }

  void run() {
    setvbuf(stdout, nullptr, _IONBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);

    const bool motionOk = motion->isCorrectlyInitialized();
    const bool gaitOk = gait->isCorrectlyInitialized();
    std::cerr << "\n========== SOCCER StableOp2 ==========\n"
              << "timeStep=" << timeStep << " ms\n"
              << "motionManager init=" << motionOk << " motionPlaying=" << motion->isMotionPlaying() << "\n"
              << "gaitManager init=" << gaitOk << "\n"
              << "head_yaw=" << (headYaw ? "ok" : "null") << " head_pitch=" << (headPitch ? "ok" : "null")
              << "\n====================================\n"
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

      if (headYaw)
        headYaw->setPosition(0.0);
      if (headPitch)
        headPitch->setPosition(-0.4);

      double t = getTime();
      double ballSpeed = 0.0;
      if (found && hadBall) {
        double dt = t - prevBallTime;
        if (dt > 1e-4)
          ballSpeed = (objX - prevBallX) / dt;
      }
      if (found) {
        prevBallX = objX;
        prevBallTime = t;
        hadBall = true;
      } else {
        hadBall = false;
      }

      int goalX = width / 2;
      bool goalSeen = detectGoal(goalX);

      int robotPixels = 0;
      detectOpponentBlue(robotPixels);

      int linePixels = 0;
      detectFieldLines(linePixels);

      int ballCloseThresh = std::max(50, width * height / 700);
      bool ballClose = found && objSize > ballCloseThresh;
      bool ballCenteredKick = found && fabs((double)objX - kickXRef) < kickCenterTol;
      bool goalAligned =
          goalSeen && fabs((double)goalX - center) < (double)kGoalCenterTolPx;

      bool canKickAgain = (t - lastKickTime) >= kKickCooldown;
      bool kickReady =
          found && (objSize > kKickMinBallPixels) && ballCenteredKick && canKickAgain;

      State state;
      if (!found)
        state = SEARCH;
      else if (robotPixels > kAvoidBlueThreshold)
        state = APPROACH;
      else if (ballClose && found && goalSeen && !goalAligned)
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
            if (robotPixels > kAvoidBlueThreshold) {
              xAmp = 0.08;
              aAmp = 0.32;
            } else {
              xAmp = 1.0;
              aAmp = -turn;
            }
            break;
          }

          case ALIGN: {
            xAmp = 0.0;
            double gNorm = 2.0 * goalX / (double)width - 1.0;
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
          std::cerr << std::fixed << std::setprecision(1) << "[vision] vx=" << ballSpeed << " gx=" << goalX
                    << " bluePx=" << robotPixels << " whitePx=" << linePixels << std::endl;
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

        int kickPage = (goalX < (int)center) ? 13 : 12;
        std::cerr << "[move] KICK playPage(" << kickPage << ") (API: isMotionPlaying not isRunning)\n"
                  << std::flush;
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
  double prevBallTime;
  bool hadBall;
  State previousState;
  double lastKickTime;

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
  static const int kLogEveryNSteps = 30;
  static const int kVisionScanStep = 6;
  static const int kKickMinBallPixels = 9000;
  static constexpr double kKickCooldown = 3.5;

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

  bool detectGoal(int &goalX) {
    const unsigned char *img = camera->getImage();
    int w = camera->getWidth();
    int h = camera->getHeight();
    const int st = kVisionScanStep;

    int count = 0;
    long sumX = 0;

    for (int x = 0; x < w; x += st) {
      for (int y = 0; y < h; y += st) {
        int r = Camera::imageGetRed(img, w, x, y);
        int g = Camera::imageGetGreen(img, w, x, y);
        int b = Camera::imageGetBlue(img, w, x, y);

        if (r > 175 && g > 175 && b < 130) {
          count++;
          sumX += x;
        }
      }
    }

    int sRef = samplesInScan(w, h, 3);
    int sCur = samplesInScan(w, h, st);
    int minGoal = std::max(20, (100 * sCur) / std::max(1, sRef));
    if (count < minGoal)
      return false;

    goalX = (int)(sumX / count);
    return true;
  }

  bool detectOpponentBlue(int &blueCount) {
    const unsigned char *img = camera->getImage();
    int w = camera->getWidth();
    int h = camera->getHeight();
    int y0 = h / 2;
    const int st = kVisionScanStep;

    blueCount = 0;
    for (int x = 0; x < w; x += st) {
      for (int y = y0; y < h; y += st) {
        int r = Camera::imageGetRed(img, w, x, y);
        int g = Camera::imageGetGreen(img, w, x, y);
        int b = Camera::imageGetBlue(img, w, x, y);
        if (b > 140 && r < 110 && g < 110)
          blueCount++;
      }
    }
    return blueCount > 0;
  }

  bool detectFieldLines(int &whiteCount) {
    const unsigned char *img = camera->getImage();
    int w = camera->getWidth();
    int h = camera->getHeight();

    whiteCount = 0;
    for (int x = 0; x < w; x += kVisionScanStep) {
      for (int y = 0; y < h; y += kVisionScanStep) {
        int r = Camera::imageGetRed(img, w, x, y);
        int g = Camera::imageGetGreen(img, w, x, y);
        int b = Camera::imageGetBlue(img, w, x, y);
        if (r > 195 && g > 195 && b > 195)
          whiteCount++;
      }
    }
    return whiteCount > 0;
  }
};

int main() {
  StableOp2 robot;
  robot.run();
  return 0;
}
