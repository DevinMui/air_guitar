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
    : onArm(false), isUnlocked(true), roll_w(0), pitch_w(0), yaw_w(0), currentPose(), emgSamples()
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
    
    // onEmgData() is called whenever a paired Myo has provided new EMG data, and EMG streaming is enabled.
    void onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
    {
        for (int i = 0; i < 8; i++) {
            emgSamples[i] = emg[i];
        }
    }
    // There are other virtual functions in DeviceListener that we could override here, like onAccelerometerData().
    // For this example, the functions overridden above are sufficient.
    // We define this function to print the current values that were updated by the on...() functions above.
    void print_emg()
    {
        // Clear the current line
        std::cout << '\r';
        // Print out the EMG data.
        for (size_t i = 0; i < emgSamples.size(); i++) {
            std::ostringstream oss;
            oss << static_cast<int>(emgSamples[i]);
            std::string emgString = oss.str();
            std::cout << '[' << emgString << std::string(4 - emgString.size(), ' ') << ']';
        }
        std::cout << std::flush;
    }
    // The values of this array is set by onEmgData() above.
    std::array<int8_t, 8> emgSamples;
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
    // Get the most recent frame and report some basic information
    const Frame frame = controller.frame();
    
    HandList hands = frame.hands();
    for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
        // Get the first hand
        const Hand hand = *hl;
        // Get the hand's normal vector and direction
        
        const Vector position = hand.palmPosition();
        // We catch any exceptions that might occur below -- see the catch statement for more details.
        try {
            
            std::cout << hand.palmPosition()[2] << std::endl;
            // First, we create a Hub with our application identifier. Be sure not to use the com.example namespace when
            // publishing your application. The Hub provides access to one or more Myos.
            myo::Hub hub("com.example.hello-myo");
            
            std::cout << "Attempting to find a Myo..." << std::endl;
            
            // Next, we attempt to find a Myo to use. If a Myo is already paired in Myo Connect, this will return that Myo
            // immediately.
            // waitForMyo() takes a timeout value in milliseconds. In this case we will try to find a Myo for 10 seconds, and
            // if that fails, the function will return a null pointer.
            myo::Myo* myo = hub.waitForMyo(10000);
            
            // If waitForMyo() returned a null pointer, we failed to find a Myo, so exit with an error message.
            if (!myo) {
                throw std::runtime_error("Unable to find a Myo!");
            }
            
            // We've found a Myo.
            std::cout << "Connected to a Myo armband!" << std::endl << std::endl;
            
            myo->setStreamEmg(myo::Myo::streamEmgEnabled);
            
            // Next we construct an instance of our DeviceListener, so that we can register it with the Hub.
            DataCollector collector;
            
            // Hub::addListener() takes the address of any object whose class inherits from DeviceListener, and will cause
            // Hub::run() to send events to all registered device listeners.
            hub.addListener(&collector);
            
            // Finally we enter our main loop.
            float pitch = collector.pitch_w;
            float yaw = collector.yaw_w;
            
            while (1) {
                // In each iteration of our main loop, we run the Myo event loop for a set number of milliseconds.
                // In this case, we wish to update our display 20 times a second, so we run for 1000/20 milliseconds.
                hub.run(1000/20);
                // After processing events, we call the print() member function we defined above to print out the values we've
                // obtained from any events that have occurred.
                //collector.print();
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
                
                // need machine learning here
                //collector.print_emg();
                
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
            
            // If a standard exception occurred, we print out its message and exit.
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << std::endl;
            std::cerr << "Press enter to continue.";
            std::cin.ignore();
        }

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
    
    // Have the sample listener receive events from the controller
    controller.addListener(listener);
    
    if (argc > 1 && strcmp(argv[1], "--bg") == 0)
        controller.setPolicy(Leap::Controller::POLICY_BACKGROUND_FRAMES);
    
    // Keep this process running until Enter is pressed
    std::cout << "Press Enter to quit..." << std::endl;
    std::cin.get();
    
    // Remove the sample listener when done
    controller.removeListener(listener);
    
    return 0;
}

