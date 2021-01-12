//libraries that are needed to run any code in C++ api
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>

//*******Include the libraries that are require for the Read_CSV function*******

//*******Change the IP_ADDRESS to connect to the robot*******
#define IP_ADDRESS "xx.xx.xxx.xxx"

//Port value of 10000 means the robot motion is in High-level control and 
//if 10001 is used than it is in Low-level control
#define PORT 10000


namespace k_api = Kinova::Api;

//set max of 30s for each action
constexpr auto TIMEOUT_DURATION = std::chrono::seconds{ 30 };

// Create an event listener that will set the promise action event to the exit value
// Will set promise to either END or ABORT
// Use finish_promise.get_future.get() to wait and get the value
std::function<void(k_api::Base::ActionNotification)>
create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise](k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch (action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            finish_promise.set_value(action_event);
            break;
        default:
            break;
        }
    };
}

// Create an event listener that will set the sent reference to the exit value
// Will set to either END or ABORT
// Read the value of returnAction until it is set
std::function<void(k_api::Base::ActionNotification)>
create_event_listener_by_ref(k_api::Base::ActionEvent& returnAction)
{
    return [&returnAction](k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch (action_event)
        {
        case k_api::Base::ActionEvent::ACTION_END:
        case k_api::Base::ActionEvent::ACTION_ABORT:
            returnAction = action_event;
            break;
        default:
            break;
        }
    };
}

//This function take the robot to the home position
//The home position is a already defined position so it search up the
//required action handle, which have all the data stored in it (basically joint angles)
/*
bool move_to_home_position(k_api::Base::BaseClient* base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list())
    {
        if (action.name() == "Home")
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0)
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
        return false;
    }
    else
    {
        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> finish_promise;
        auto finish_future = finish_promise.get_future();
        auto promise_notification_handle = base->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions()
        );

        // Execute action
        base->ExecuteActionFromReference(action_handle);

        // Wait for future value from promise
        const auto status = finish_future.wait_for(TIMEOUT_DURATION);
        base->Unsubscribe(promise_notification_handle);

        if (status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
            return false;
        }
        const auto promise_event = finish_future.get();
        return true;
    }
}*/

//*******Write the function called move_to_zero_position or move_to_retract_position depending on your UCID*******
//*******Use the move_to_home_position function to write move_to_zero_position or move_to_retract_position*******
//*******Start here*******








//*******Function end here*******

bool angular_action_movement(k_api::Base::BaseClient* base, vector<float> angles)
{
    std::cout << "Starting angular action movement ..." << std::endl;

    auto action = k_api::Base::Action();
    action.set_name("Example angular action movement");
    action.set_application_data("");

    auto reach_joint_angles = action.mutable_reach_joint_angles();//mutable_reach_joint_angles allow to set the joint angles of the robot
    auto joint_angles = reach_joint_angles->mutable_joint_angles();

    auto actuator_count = base->GetActuatorCount();//number of joints

    //setting up the joint angles
    for (size_t i = 0; i < actuator_count.count(); ++i)
    {
        auto joint_angle = joint_angles->add_joint_angles();
        joint_angle->set_joint_identifier(i);
        joint_angle->set_value(angles.at(i));
    }

    // Connect to notification action topic
    // (Promise alternative)
    // See cartesian examples for Reference alternative
    std::promise<k_api::Base::ActionEvent> finish_promise;
    auto finish_future = finish_promise.get_future();
    auto promise_notification_handle = base->OnNotificationActionTopic(
        create_event_listener_by_promise(finish_promise),
        k_api::Common::NotificationOptions()
    );

    //perform the action 
    std::cout << "Executing action" << std::endl;
    base->ExecuteAction(action);

    std::cout << "Waiting for movement to finish ..." << std::endl;

    // Wait for future value from promise
    // (Promise alternative)
    // See cartesian examples for Reference alternative
    const auto status = finish_future.wait_for(TIMEOUT_DURATION);
    base->Unsubscribe(promise_notification_handle);

    if (status != std::future_status::ready)
    {
        std::cout << "Timeout on action notification wait" << std::endl;
        return false;
    }
    const auto promise_event = finish_future.get();

    std::cout << "Angular movement completed" << std::endl;
    std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl;

    return true;
}

bool example_cartesian_action_movement(k_api::Base::BaseClient* base, vector<float> position, vector<float> orientation, float velocity)
{
    std::cout << "Starting Cartesian action movement ..." << std::endl;
    auto action = k_api::Base::Action();
    action.set_name("Example Cartesian action movement");
    action.set_application_data("");

    auto constrained_pose = action.mutable_reach_pose();//mutable_reach_pose() allows to set the cartesian position of the end effector
    auto pose = constrained_pose->mutable_target_pose();
    pose->set_x(position.at(0));                // x (meters)
    pose->set_y(position.at(1));          // y (meters)
    pose->set_z(position.at(2));          // z (meters)
    pose->set_theta_x(orientation.at(0));    // theta x (degrees)
    pose->set_theta_y(orientation.at(1));    // theta y (degrees)
    pose->set_theta_z(orientation.at(2));    // theta z (degrees)

    // Connect to notification action topic
    // (Reference alternative)
    // See angular examples for Promise alternative
    k_api::Base::ActionEvent event = k_api::Base::ActionEvent::UNSPECIFIED_ACTION_EVENT;
    auto reference_notification_handle = base->OnNotificationActionTopic(
        create_event_listener_by_ref(event),
        k_api::Common::NotificationOptions()
    );
    //translation speed
    auto speed_constrianed = constrained_pose->mutable_constraint();
    auto speed = speed_constrianed->mutable_speed();
    speed->set_translation(velocity);
    
    std::cout << "Executing action" << std::endl;
    base->ExecuteAction(action);

    std::cout << "Waiting for movement to finish ..." << std::endl;

    // Wait for reference value to be set
    // (Reference alternative)
    // See angular examples for Promise alternative
    // Set a timeout after 20s of wait
    const auto timeout = std::chrono::system_clock::now() + TIMEOUT_DURATION;
    while (event == k_api::Base::ActionEvent::UNSPECIFIED_ACTION_EVENT &&
        std::chrono::system_clock::now() < timeout)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    base->Unsubscribe(reference_notification_handle);

    if (event == k_api::Base::ActionEvent::UNSPECIFIED_ACTION_EVENT)
    {
        std::cout << "Timeout on action notification wait" << std::endl;
        return false;
    }

    std::cout << "Cartesian movement completed" << std::endl;
    std::cout << "Reference value : " << k_api::Base::ActionEvent_Name(event) << std::endl;

    return true;
}
//*******Write the Read_CSV function*******
//*******Start here*******







//*******Function end here*******

int main(int argc, char** argv) {
	vector<vector<float>> angle;
    vector<vector<float>> position;
    vector<vector<float>> orientation;
    vector<float> gripper_position;
    vector<float> translation_speed;
    vector<int> action_sequence;
	
	//*******Add the other require variable below******* 
	
	
	
	//*******Add the address of the CSV below******* 
	string Filename = "Address of CSV";
	
	
	 // Create API objects
    auto error_callback = [](k_api::KError err) { cout << "_________ callback error _________" << err.toString(); };
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(IP_ADDRESS, PORT);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
	//*******Add the username******* 
    create_session_info.set_username("xxxxxxxxxxx");
    //*******Add the password*******
	create_session_info.set_password("xxxxxxxxxxxx");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating session for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    std::cout << "Session created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router);
    //gripper services
    k_api::Base::GripperCommand gripper_command;
    gripper_command.set_mode(k_api::Base::GRIPPER_POSITION);
    auto finger = gripper_command.mutable_gripper()->add_finger();
    finger->set_finger_identifier(1);
	
	
	Read_CSV(Filename_Clone, angle, position, orientation, gripper_position, translation_speed, action_sequence);
	
	bool success = true;
	//*******Write the loop that perform the action*******
	//*******start here*******
	//use the variable "success" to know that all the action are perform
	
	
	
	
	
	
	
	//*******end the loop here*******
	
	//closing the session
    session_manager->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();

    // Destroy the API
    delete base;
    delete session_manager;
    delete router;
    delete transport;

    return success ? 0 : 1;
}