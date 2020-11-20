import React from 'react'
import {ROS_REMOTE} from "../../../common/host"
import {Card, CardHeader, Col, Row} from "reactstrap";
import NavigationBar from "../../../navbar";
import ROSLIB from 'roslib'

class LiveContainer extends React.Component {

    constructor(props, context) {
        super(props, context);

        this.state = {

            ros: new ROSLIB.Ros({url: 'ws://' + ROS_REMOTE.address + ':' + ROS_REMOTE.ws_port}),

            ros_connected: false,

            topic: {
                armed: {
                    value: false,
                    path: "/Armed",
                    type: "std_msgs/Bool"
                },
                control: {
                    value: {
                        roll: 0,
                        pitch: 0,
                        yaw: 0,
                        throttle: 0
                    },
                    path: "/Control",
                    type: "drone/ControlAxes"
                },
                sonar: {
                    value: 0,
                    path: "/SonarReading",
                    type: "std_msgs/Float32"
                },
                attitude: {
                    value: {
                        roll: 0,
                        pitch: 0,
                        yaw: 0,
                        percentage: 0,
                        power: 0,
                        camera_angle: 0
                    },
                    path: "/CraftAttitude",
                    type: "drone/Attitude"
                }
            }
        }
    }

    getTopics() {
        let self = this

        new ROSLIB.Topic({
            ros: this.state.ros,
            name: this.state.topic.control.path,
            messageType: this.state.topic.control.type
        }).subscribe(function (message) {
            if (message) {
                let state = self.state
                state['topic']['control']['value'] = {
                    roll: message.axis[0],
                    pitch: message.axis[1],
                    throttle: message.axis[2],
                    yaw: message.axis[3]
                }
                self.setState(state)
            }
        });

        new ROSLIB.Topic({
            ros: this.state.ros,
            name: this.state.topic.attitude.path,
            messageType: this.state.topic.attitude.type
        }).subscribe(function (message) {
            if (message) {
                let state = self.state
                state['topic']['attitude']['value'] = {
                    roll: message.roll,
                    pitch: message.pitch,
                    yaw: message.yaw,
                    percentage: message.percentage,
                    power: message.power,
                    camera_angle: message.camera_angle
                }
                self.setState(state)
            }
        });
    }

    componentDidMount() {
        let self = this

        this.state.ros.on('connection', function () {
            console.log('Connected to websocket on ' + ROS_REMOTE.address + ':' + ROS_REMOTE.ws_port);
            self.getTopics()
            let state = self.state
            state['ros_connected'] = true
            self.setState(state)
        });

        this.state.ros.on('error', function (error) {
            console.log('Error connecting to websocket server: ', error);
        });

        this.state.ros.on('close', function () {
            console.log('Connection to websocket server closed.');
            let state = self.state
            state['ros_connected'] = false
            self.setState(state)
        });

    }


    render() {
        return (
            <div>
                <NavigationBar/>
                <Col sm={{size: '4', offset: 0}} style={{marginLeft: "10px"}}>

                    <CardHeader style={{marginTop: "10px"}}>
                        <strong>{"Live Aerial Video Feed - " + this.state.topic.attitude.value.camera_angle + " deg"}</strong>
                    </CardHeader>

                    <Card style={{padding: "5px"}}>
                        <img width={345}
                             height={200}
                             src={"http://" + ROS_REMOTE.address + ":" + ROS_REMOTE.video_port + "/" + ROS_REMOTE.video_path}
                             alt={"Live Video Feed"}/>
                    </Card>

                </Col>


            </div>
        );
    }
}

export default LiveContainer