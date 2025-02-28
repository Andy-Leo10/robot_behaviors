### Main Structure
The root element specifies the format and the main tree to execute:
```xml
<root BTCPP_format="4" main_tree_to_execute="MainTree">
```

### Main Behavior Tree
The `MainTree` behavior tree consists of a sequence of subtasks:
```xml
<BehaviorTree ID="MainTree">
    <Sequence>
        <SubTree ID="SelfLocalization" name="self_localization"/>
        <SubTree ID="NavigateAndInspect" name="west" x="0.357" y="-3.596" theta="0.017"/>
        <SubTree ID="NavigateAndInspect" name="east" x="1.026" y="2.507" theta="0.130"/>
    </Sequence>
</BehaviorTree>
```
- **SelfLocalization**: A subtree for localizing the robot.
- **NavigateAndInspect**: Subtrees for navigating to specific coordinates and inspecting objects.

### Self Localization SubTree
This subtree retries the localization process up to three times:
```xml
<BehaviorTree ID="SelfLocalization">
    <RetryUntilSuccessful num_attempts="3">
        <Sequence>
            <ReinitializeLocalization name="ReinitializeLocalization" service_name="/reinitialize_global_localization"/>
            <Spin name="Spin" action_name="spin" target_yaw="8.88" time_allowance_sec="15"/>
        </Sequence>
    </RetryUntilSuccessful>
</BehaviorTree>
```
- **ReinitializeLocalization**: Reinitializes the robot's localization.
- **Spin**: Spins the robot to aid in localization.

### Navigate and Inspect SubTree
This subtree continuously navigates and inspects objects:
```xml
<BehaviorTree ID="NavigateAndInspect">
    <RetryUntilSuccessful num_attempts="-1">
        <Sequence>
            <ReceiveString name="ReceiveString" topic_name="my_object_detected" message="{detection}"/>
            <SubTree ID="HandleDetection" detection="{detection}"/>
            <Parallel success_count="1" failure_count="1">
                <GoToPose name="GoToPose" action_name="navigate_to_pose" x="{x}" y="{y}" theta="{theta}"/>
                <Repeat num_cycles="-1">
                    <Delay delay_msec="1000">
                        <Sequence>
                            <ReceiveString name="ReceiveString" topic_name="my_object_detected" message="{detection}"/>
                            <SendString name="SendString" topic_name="my_topic" msg="{detection}"/>
                            <SubTree ID="DetectionSwitch" detection="{detection}"/>
                        </Sequence>
                    </Delay>
                </Repeat>
            </Parallel>
        </Sequence>
    </RetryUntilSuccessful>
</BehaviorTree>
```
It succeeds when the robot navigates to the specified pose, because the `success_count` is set to 1 in the `Parallel` node. The `Repeat` node continuously inspects objects by receiving messages from the `my_object_detected` topic.
- **ReceiveString**: Receives object detection messages.
- **HandleDetection**: Handles actions based on the detected object.
- **GoToPose**: Navigates to a specified pose.
- **DetectionSwitch**: Switches actions based on the detected object.

### Handle Detection SubTree
This subtree handles actions based on the detected object:
```xml
<BehaviorTree ID="HandleDetection">
    <Sequence>
        <SendString name="NotifyPerson" topic_name="my_topic" msg="Performing wait ....................." _skipIf="detection != 'person'"/>
        <Wait name="Wait" action_name="wait" time_sec="5" _skipIf="detection != 'person'"/>
        <SendString name="NotifyCar" topic_name="my_topic" msg="Car Action ....................." _skipIf="detection != 'car'"/>
        <SendString name="NotifyBicycle" topic_name="my_topic" msg="Bicycle Action ....................." _skipIf="detection != 'bicycle'"/>
        <SendString name="NotifyDefault" topic_name="my_topic" msg="Default Action ....................." _skipIf="detection != 'nothing'"/>
    </Sequence>
</BehaviorTree>
```
- **SendString**: Sends messages based on the detected object.
- **Wait**: Waits for a specified time if a person is detected.

### Detection Switch SubTree
This subtree switches actions based on the detected object:
```xml
<BehaviorTree ID="DetectionSwitch">
    <Switch3 name="SimpleSwitch" variable="{detection}" case_1="person" case_2="car" case_3="bicycle">
        <Sequence name="case_1">
            <ForceFailure>
                <SendString name="NotifyPerson" topic_name="my_topic" msg="Person detected --------------------"/>
            </ForceFailure>
        </Sequence>
        <Sequence name="case_2">
            <SendString name="NotifyCar" topic_name="my_topic" msg="Car detected"/>
        </Sequence>
        <Sequence name="case_3">
            <SendString name="NotifyBicycle" topic_name="my_topic" msg="Bicycle detected"/>
        </Sequence>
        <Sequence name="default">
            <SendString name="NotifyDefault" topic_name="my_topic" msg="Default case"/>
        </Sequence>
    </Switch3>
</BehaviorTree>
```
As shown, when a person is detected, a failure is forced, because the robot should not perform any action. Then, once the detection is different from a person, the robot performs again the `GoToPose` action.
- **Switch3**: Switches between different sequences based on the detected object.

### Node Models
This section describes the node models used in the behavior tree:
```xml
<TreeNodesModel>
    <Action ID="GoToPose" editable="true">
        <input_port name="action_name"/>
        <input_port name="x"/>
        <input_port name="y"/>
        <input_port name="theta"/>
    </Action>
    <Condition ID="ReceiveString" editable="true">
        <input_port name="topic_name"/>
        <output_port name="message"/>
    </Condition>
    <Condition ID="SendString" editable="true">
        <input_port name="topic_name"/>
        <input_port name="msg"/>
    </Condition>
    <Action ID="Spin" editable="true">
        <input_port name="action_name"/>
        <input_port name="target_yaw"/>
        <input_port name="time_allowance_sec"/>
    </Action>
    <Action ID="Wait" editable="true">
        <input_port name="action_name"/>
        <input_port name="time_sec"/>
    </Action>
    <Action ID="ReinitializeLocalization" editable="true">
        <input_port name="service_name"/>
    </Action>
</TreeNodesModel>
```
This is necessary to define the input and output ports for each node for `Groot2`.
- **GoToPose**: Action node for navigating to a pose.
- **ReceiveString**: Condition node for receiving strings.
- **SendString**: Condition node for sending strings.
- **Spin**: Action node for spinning the robot.
- **Wait**: Action node for waiting.
- **ReinitializeLocalization**: Action node for reinitializing localization.
