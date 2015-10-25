#define _USE_MATH_DEFINES
#include <cmath>
#include <iostream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <cstring>
#include <array>
#include <sstream>
#include <stdexcept>

// The only file that needs to be included to use the Myo C++ SDK is myo.hpp.
#include <myo/myo.hpp>
#include <SFML/audio.hpp>
#include <iostream>
#include <cstring>
#include <math.h>
#include "Leap.h"

using namespace Leap;

class DataCollector : public myo::DeviceListener {
public:
    DataCollector()
    : onArm(false), isUnlocked(true), roll_w(0), pitch_w(0), yaw_w(0), currentPose()
    {
    }
    
    // onUnpair() is called whenever the Myo is disconnected from Myo Connect by the user.
    void onUnpair(myo::Myo* myo, uint64_t timestamp)
    {
        // We've lost a Myo.
        // Let's clean up some leftover state.
        roll_w = 0;
        pitch_w = 0;
        yaw_w = 0;
        onArm = false;
        isUnlocked = false;
    }
    
    // onOrientationData() is called whenever the Myo device provides its current orientation, which is represented
    // as a unit quaternion.
    void onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float>& quat)
    {
        using std::atan2;
        using std::asin;
        using std::sqrt;
        using std::max;
        using std::min;
        
        // Calculate Euler angles (roll, pitch, and yaw) from the unit quaternion.
        float roll = atan2(2.0f * (quat.w() * quat.x() + quat.y() * quat.z()),
                           1.0f - 2.0f * (quat.x() * quat.x() + quat.y() * quat.y()));
        float pitch = asin(max(-1.0f, min(1.0f, 2.0f * (quat.w() * quat.y() - quat.z() * quat.x()))));
        float yaw = atan2(2.0f * (quat.w() * quat.z() + quat.x() * quat.y()),
                          1.0f - 2.0f * (quat.y() * quat.y() + quat.z() * quat.z()));
        
        // Convert the floating point angles in radians to a scale from 0 to 18.
        roll_w = static_cast<int>((roll + (float)M_PI)/(M_PI * 2.0f) * 18);
        pitch_w = static_cast<int>((pitch + (float)M_PI/2.0f)/M_PI * 18);
        yaw_w = static_cast<int>((yaw + (float)M_PI)/(M_PI * 2.0f) * 18);
    }
    
    // onPose() is called whenever the Myo detects that the person wearing it has changed their pose, for example,
    // making a fist, or not making a fist anymore.
    void onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
    {
        currentPose = pose;
        
        
        // Tell the Myo to stay unlocked until told otherwise. We do that here so you can hold the poses without the
        // Myo becoming locked.
        myo->unlock(myo::Myo::unlockHold);
        
        // Notify the Myo that the pose has resulted in an action, in this case changing
        // the text on the screen. The Myo will vibrate.
        myo->notifyUserAction();
        
    }
    
    // onArmSync() is called whenever Myo has recognized a Sync Gesture after someone has put it on their
    // arm. This lets Myo know which arm it's on and which way it's facing.
    void onArmSync(myo::Myo* myo, uint64_t timestamp, myo::Arm arm, myo::XDirection xDirection, float rotation,
                   myo::WarmupState warmupState)
    {
        onArm = true;
        whichArm = arm;
    }
    
    // onArmUnsync() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
    // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
    // when Myo is moved around on the arm.
    void onArmUnsync(myo::Myo* myo, uint64_t timestamp)
    {
        onArm = false;
    }
    
    // onUnlock() is called whenever Myo has become unlocked, and will start delivering pose events.
    void onUnlock(myo::Myo* myo, uint64_t timestamp)
    {
        isUnlocked = true;
    }
    
    // onLock() is called whenever Myo has become locked. No pose events will be sent until the Myo is unlocked again.
    void onLock(myo::Myo* myo, uint64_t timestamp)
    {
        isUnlocked = false;
    }
    
    // There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
    // For this example, the functions overridden above are sufficient.
    
    // These values are set by onArmSync() and onArmUnsync() above.
    bool onArm;
    myo::Arm whichArm;
    
    // This is set by onUnlocked() and onLocked() above.
    bool isUnlocked;
    
    // These values are set by onOrientationData() and onPose() above.
    int roll_w, pitch_w, yaw_w;
    myo::Pose currentPose;
};


class SampleListener : public Listener {
public:
    virtual void onInit(const Controller&);
    virtual void onConnect(const Controller&);
    virtual void onDisconnect(const Controller&);
    virtual void onExit(const Controller&);
    virtual void onFrame(const Controller&);
    virtual void onFocusGained(const Controller&);
    virtual void onFocusLost(const Controller&);
    virtual void onDeviceChange(const Controller&);
    virtual void onServiceConnect(const Controller&);
    virtual void onServiceDisconnect(const Controller&);
    
private:
};

void SampleListener::onInit(const Controller& controller) {
    std::cout << "Initialized" << std::endl;
}

void SampleListener::onConnect(const Controller& controller) {
    std::cout << "Connected" << std::endl;
    controller.enableGesture(Gesture::TYPE_CIRCLE);
    controller.enableGesture(Gesture::TYPE_KEY_TAP);
    controller.enableGesture(Gesture::TYPE_SCREEN_TAP);
    controller.enableGesture(Gesture::TYPE_SWIPE);
}

void SampleListener::onDisconnect(const Controller& controller) {
    // Note: not dispatched when running in a debugger.
    std::cout << "Disconnected" << std::endl;
}

void SampleListener::onExit(const Controller& controller) {
    std::cout << "Exited" << std::endl;
}

void SampleListener::onFrame(const Controller& controller) {
    const Frame frame = controller.frame();
    HandList hands = frame.hands();
    try {
        myo::Hub hub("io.github.devinmui.finger");
        std::cout << "Attempting to find a Myo..." << std::endl;
            
        myo::Myo* myo = hub.waitForMyo(10000);
            
        if (!myo) {
            throw std::runtime_error("Unable to find a Myo!");
        }
            
        std::cout << "Connected to a Myo armband!" << std::endl << std::endl;
    
        
        DataCollector collector;
            
        hub.addListener(&collector);
            
        float pitch = collector.pitch_w;
        float yaw = collector.yaw_w;
                
        for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); hl++) {
            const Hand hand = *hl;
            hub.run(1000/20);
            double foo = hand.palmPosition()[2];
            std::cout << std::to_string(foo) << std::endl;
            std::string pose = collector.currentPose.toString();
            float init_pitch = collector.pitch_w;
            float init_yaw = collector.yaw_w;
            float move_pitch = 0;
            float move_yaw = 0;
            if(init_pitch - pitch < 0) {
                move_pitch = (init_pitch - pitch) * -1;
            } else {
                move_pitch = init_pitch - pitch; // calculate the movement of the pitch
            }
            if(init_yaw - yaw < 0) {
                move_yaw = (init_yaw - yaw) * -1; // calculate the movement of the yaw
            } else {
                move_yaw = init_yaw - yaw;
            }
        
            // after calculations
            if(move_pitch >= 1) {
                std::cout << "YES" << std::endl;
            } else {
                std::cout << "NO" << std::endl;
            }
                    
            //std::cout << "pitch: " << init_pitch << ", yaw: "<< init_yaw << std::endl;
            pitch = init_pitch; // pitch should be around ~ 5+ difference
            yaw = init_yaw; // yaw should be 1 - 2 difference
            // whatever might not need yaw or roll just do pitch
                    
            if(pose == "fist"){
                std::cout << "FISTBUMP!" << std::endl;
                // play music based on calculations from leap motion
            } else {
                std::cout << "No fistbump :(" << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::cerr << "Press enter to continue.";
        std::cin.ignore()
        ;
        //return -1;
    }
}

    




void SampleListener::onFocusGained(const Controller& controller) {
    std::cout << "Focus Gained" << std::endl;
}

void SampleListener::onFocusLost(const Controller& controller) {
    std::cout << "Focus Lost" << std::endl;
}

void SampleListener::onDeviceChange(const Controller& controller) {
    std::cout << "Device Changed" << std::endl;
    const DeviceList devices = controller.devices();
    
    for (int i = 0; i < devices.count(); ++i) {
        std::cout << "id: " << devices[i].toString() << std::endl;
        std::cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << std::endl;
    }
}

void SampleListener::onServiceConnect(const Controller& controller) {
    std::cout << "Service Connected" << std::endl;
}

void SampleListener::onServiceDisconnect(const Controller& controller) {
    std::cout << "Service Disconnected" << std::endl;
}

int main(int argc, char** argv)
{
    SampleListener listener;
    Controller controller;
    
    
    try {
        myo::Hub hub("io.github.devinmui.finger");
        std::cout << "Attempting to find a Myo..." << std::endl;
        
        myo::Myo* myo = hub.waitForMyo(10000);
        
        if (!myo) {
            throw std::runtime_error("Unable to find a Myo!");
        }
        
        std::cout << "Connected to a Myo armband!" << std::endl << std::endl;
        
        
        DataCollector collector;
        
        hub.addListener(&collector);
        
        float pitch = collector.pitch_w;
        float yaw = collector.yaw_w;
        while(1){
            const Frame frame = controller.frame();
            HandList hands = frame.hands();
            for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); hl++) {
                const Hand hand = *hl;
                double foo = hand.palmPosition()[2];
                std::cout << std::to_string(foo) << std::endl;
            }
            hub.run(1000/20);
            std::string pose = collector.currentPose.toString();
            float init_pitch = collector.pitch_w;
            float init_yaw = collector.yaw_w;
            float move_pitch = 0;
            float move_yaw = 0;
            if(init_pitch - pitch < 0) {
                move_pitch = (init_pitch - pitch) * -1;
            } else {
                move_pitch = init_pitch - pitch; // calculate the movement of the pitch
            }
            if(init_yaw - yaw < 0) {
                move_yaw = (init_yaw - yaw) * -1; // calculate the movement of the yaw
            } else {
                move_yaw = init_yaw - yaw;
            }
            
            // after calculations
            if(move_pitch >= 1) {
                std::cout << "YES" << std::endl;
            } else {
                std::cout << "NO" << std::endl;
            }
            
            //std::cout << "pitch: " << init_pitch << ", yaw: "<< init_yaw << std::endl;
            pitch = init_pitch; // pitch should be around ~ 5+ difference
            yaw = init_yaw; // yaw should be 1 - 2 difference
            // whatever might not need yaw or roll just do pitch
            
            if(pose == "fist"){
                std::cout << "FISTBUMP!" << std::endl;
                // play music based on calculations from leap motion
            } else {
                std::cout << "No fistbump :(" << std::endl;
            }
        
        }
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
            std::cerr << "Press enter to continue.";
            std::cin.ignore()
            ;
            return -1;
        }
    
}

