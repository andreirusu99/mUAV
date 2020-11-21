import React from 'react'
import {ROS_REMOTE} from "../../../common/host"
import {Badge, Card, CardHeader, Col, Progress, Row} from "reactstrap";
import NavigationBar from "../../../navbar";
import ROSLIB from 'roslib'
import {Slider} from "@material-ui/core";

class DashboardContainer extends React.Component {

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
                        roll: 1500,
                        pitch: 1500,
                        yaw: 1500,
                        throttle: 1000
                    },
                    path: "/Control",
                    type: "drone/ControlAxes"
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
                },
                sonar: {
                    value: 0,
                    path: "/SonarReading",
                    type: "std_msgs/Float32"
                },
                gps: {
                    value: 0,
                    path: "/GPSReading",
                    // type: "std_msgs/Float32"
                }

            }
        }
    }

    getTopics() {
        let self = this

        new ROSLIB.Topic({
            ros: this.state.ros,
            name: this.state.topic.armed.path,
            messageType: this.state.topic.armed.type
        }).subscribe(function (message) {
            if (message) {
                let state = self.state
                state['topic']['armed']['value'] = message.data
                if (message.data === false) {
                    state['topic']['control']['value']['throttle'] = 1000
                    state['topic']['control']['value']['roll'] = 1500
                    state['topic']['control']['value']['pitch'] = 1500
                    state['topic']['control']['value']['yaw'] = 1500
                }
                self.setState(state)
            }
        });

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
                if (message.axis[2] < 1040)
                    state['topic']['control']['value']['throttle'] = 1040

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

                <Row>

                    <Col sm={{size: '4', offset: 0}} style={{marginLeft: "10px", marginTop: "10px"}}>

                        <Row>
                            <Col className={"text-center"}>
                                <Badge
                                    style={{fontSize: "18px", width: "150px", height: "30px"}}
                                    color={this.state.topic.armed.value === true ? 'danger' : 'success'}>
                                    {this.state.topic.armed.value === true ? 'ARMED' : 'DISARMED'}
                                </Badge>
                            </Col>
                        </Row>

                        <Row>
                            <Col>
                                <CardHeader className={"text-center"} style={{marginTop: "10px"}}>
                                    <strong>{"Live Aerial Video Feed @ " + this.state.topic.attitude.value.camera_angle + "°"}</strong>
                                </CardHeader>
                                <Card style={{paddingLeft: "5px", paddingTop: "5px", paddingBottom: "5px"}}>

                                    <img
                                        width={390}
                                        height={220}
                                        src={"http://" + ROS_REMOTE.address + ":" + ROS_REMOTE.video_port + "/" + ROS_REMOTE.video_path}
                                        alt={"Live Video Feed"}/>
                                </Card>
                            </Col>
                        </Row>

                        <Row>
                            <Col>

                                <CardHeader className={"text-center"} style={{marginTop: "10px"}}>
                                    <strong>{"Power Management"}</strong>
                                </CardHeader>

                                <Card style={{padding: "15px", paddingLeft: '25px', paddingRight: '25px'}}>

                                    <div style={{fontSize: "18px", marginTop: 'px'}}
                                         className={"text-center"}>

                                        <div style={{textAlign: "left", marginBottom: "-25px"}}>
                                            {"Power"}
                                        </div>

                                        <strong>{this.state.topic.attitude.value.power + " W"}</strong>
                                        <Progress
                                            color={"danger"}
                                            value={this.state.topic.attitude.value.power / 2}/>
                                    </div>

                                    <div style={{fontSize: "18px", marginTop: '15px'}}
                                         className={"text-center"}>
                                        <div style={{textAlign: "left", marginBottom: "-25px"}}>
                                            {"Battery"}
                                        </div>
                                        <strong>{this.state.topic.attitude.value.percentage + "%"}</strong>

                                        <Progress
                                            color={"success"}
                                            value={this.state.topic.attitude.value.percentage}/>
                                    </div>
                                </Card>
                            </Col>
                        </Row>

                    </Col>

                    <Col sm={{size: '3', offset: 0}} style={{marginTop: "40px"}}>

                        <Row>

                            <Col>
                                <CardHeader className={"text-center"} style={{marginTop: "10px"}}>
                                    <strong>{"Control Axes"}</strong>
                                </CardHeader>

                                <Card style={{padding: "15px", fontSize: "16px"}}>

                                    <div className={"text-center"} style={{width: "100%", marginTop: "5px"}}>
                                        <div style={{textAlign: "left", marginBottom: "-25px"}}>
                                            {"Throttle"}
                                        </div>
                                        <strong>{Math.round(((this.state.topic.control.value.throttle - 1000) / 1000) * 100) + "%"}</strong>
                                        <Progress
                                            color={"info"}
                                            value={Math.round(((this.state.topic.control.value.throttle - 1000) / 1000) * 100)}>
                                            {this.state.topic.control.value.throttle}
                                        </Progress>
                                    </div>

                                    <div className={"text-center"}
                                         style={{width: "100%", marginTop: "15px"}}>

                                        <div style={{textAlign: "left", marginBottom: "-25px"}}>
                                            {"Roll"}
                                        </div>
                                        <strong>{Math.round(((this.state.topic.control.value.roll - 1000) * 200 / 1000 - 100)) + "%"}</strong>

                                        {this.state.topic.armed.value &&
                                        <strong>
                                            {Math.round(((this.state.topic.control.value.roll - 1000) * 200 / 1000 - 100)) > 0
                                                ? "→"
                                                : "←"}
                                        </strong>}

                                        <Progress
                                            color={"info"}
                                            value={Math.round(((this.state.topic.control.value.roll - 1000) / 1000) * 100)}>
                                            {this.state.topic.control.value.roll}
                                        </Progress>

                                    </div>

                                    <div className={"text-center"}
                                         style={{width: "100%", marginTop: "15px"}}>

                                        <div style={{textAlign: "left", marginBottom: "-25px"}}>
                                            {"Pitch"}
                                        </div>
                                        <strong>{Math.round(((this.state.topic.control.value.pitch - 1000) * 200 / 1000 - 100)) + "%"}
                                        </strong>

                                        {this.state.topic.armed.value &&
                                        <strong>
                                            {Math.round(((this.state.topic.control.value.pitch - 1000) * 200 / 1000 - 100)) > 0
                                                ? "↑"
                                                : "↓"}
                                        </strong>}

                                        <Progress
                                            color={"info"}
                                            value={Math.round(((this.state.topic.control.value.pitch - 1000) / 1000) * 100)}>
                                            {this.state.topic.control.value.pitch}
                                        </Progress>
                                    </div>

                                </Card>
                            </Col>
                        </Row>

                        <Row>

                            <Col>
                                <CardHeader className={"text-center"} style={{marginTop: "10px"}}>
                                    <strong>{"UAV Orientation"}</strong>
                                </CardHeader>

                                <Card style={{
                                    padding: "15px",
                                    fontSize: "16px",
                                    paddingLeft: "20px",
                                    paddingRight: "20px",
                                }}>

                                    <div className={"text-center"}
                                         style={{width: "100%", marginTop: "5px"}}>

                                        <strong>{this.state.topic.attitude.value.roll.toFixed(1) + "°"}</strong>
                                        {" Roll"}

                                        <Slider
                                            track={false}
                                            marks={[
                                                {value: 0, label: '20°'},
                                                {value: 12.5, label: '15°'},
                                                {value: 25, label: '10°'},
                                                {value: 37.5, label: '5°'},
                                                {value: 50, label: '0°'},
                                                {value: 62.5, label: '5°'},
                                                {value: 75, label: '10°'},
                                                {value: 87.5, label: '15°'},
                                                {value: 100, label: '20°'},
                                            ]}
                                            value={((this.state.topic.attitude.value.roll + 20) * 100 / 40)}
                                        />
                                    </div>

                                    <div className={"text-center"}
                                         style={{width: "100%", marginTop: "5px"}}>

                                        <strong>{this.state.topic.attitude.value.pitch.toFixed(1) + "°"}</strong>
                                        {" Pitch"}

                                        <Slider
                                            track={false}
                                            marks={[
                                                {value: 0, label: '20°'},
                                                {value: 12.5, label: '15°'},
                                                {value: 25, label: '10°'},
                                                {value: 37.5, label: '5°'},
                                                {value: 50, label: '0°'},
                                                {value: 62.5, label: '5°'},
                                                {value: 75, label: '10°'},
                                                {value: 87.5, label: '15°'},
                                                {value: 100, label: '20°'},
                                            ]}
                                            value={((this.state.topic.attitude.value.pitch + 20) * 100 / 40)}
                                        />
                                    </div>

                                </Card>
                            </Col>
                        </Row>

                    </Col>

                </Row>


            </div>
        );
    }
}

export default DashboardContainer