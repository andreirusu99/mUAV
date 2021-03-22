import React from 'react'
import {ROS_REMOTE} from '../../../common/host'
import {Card, Col, Row} from 'reactstrap';
import NavigationBar from '../../../navbar';
import ROSLIB from 'roslib'
import Chip from '@material-ui/core/Chip';
import ErrorSharpIcon from '@material-ui/icons/ErrorSharp';
import CheckCircleSharpIcon from '@material-ui/icons/CheckCircleSharp';
import SignalWifi3BarIcon from '@material-ui/icons/SignalWifi3Bar';
import SignalWifiOffIcon from '@material-ui/icons/SignalWifiOff';
import GpsOffIcon from '@material-ui/icons/GpsOff';
import GpsFixedIcon from '@material-ui/icons/GpsFixed';
import VideocamIcon from "@material-ui/icons/Videocam";
import VideocamOffIcon from "@material-ui/icons/VideocamOff";
import Paper from "@material-ui/core/Paper";
import {RemoveScroll} from "react-remove-scroll";

class CrowdDataContainer extends React.Component {

    constructor(props, context) {
        super(props, context);

        this.state = {

            ros: new ROSLIB.Ros({url: 'ws://' + ROS_REMOTE.address + ':' + ROS_REMOTE.ws_port}),

            ros_connected: false,

            topic: {
                compute: {
                    value :{
                        people: 0,
                        neck_breathers: 0,
                        density: 0.0,
                        area: 0.0
                    },
                    path:'/Compute',
                    type: 'drone/ComputeMsg'
                },
                armed: {
                    value: false,
                    path: '/Armed',
                    type: 'std_msgs/Bool'
                },
                control: {
                    value: {
                        roll: 1500,
                        pitch: 1500,
                        yaw: 1500,
                        throttle: 1000
                    },
                    path: '/ControlSlow',
                    type: 'drone/ControlAxes'
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
                    path: '/CraftAttitude',
                    type: 'drone/Attitude'
                },
                sonar: {
                    value: 0,
                    path: '/SonarReading',
                    type: 'std_msgs/Float32'
                },
                gps: {
                    value: {
                        lat: 46.766715,//N
                        lng: 23.618749, //E
                        fix: 0,
                        speed: 0.0
                    },
                    path: '/GPS',
                    type: 'drone/GPSinfo'
                },
                bmp: {
                    value: {
                        rel_alt: 0.0,
                        abs_alt: 0.0,
                        temp: 0.0
                    },
                    path: '/Altitude',
                    type: 'drone/Altitude'
                },
                run: {
                    value: {
                        runtime: 0.0,
                        detection_started: false
                    },
                    path: '/Run',
                    type: 'drone/RunInfo'
                }
            }
        }
    }

    initTopics() {
        let self = this

        // subscribe to arming
        new ROSLIB.Topic({
            ros: this.state.ros,
            name: this.state.topic.armed.path,
            messageType: this.state.topic.armed.type
        }).subscribe(function (message) {
            if (message) {
                let state = self.state
                state['topic']['armed']['value'] = message.data
                if (message.data === false) {
                    state['topic']['control']['value'] = {
                        roll: 1500,
                        pitch: 1500,
                        yaw: 1500,
                        throttle: 1000
                    }
                }
                self.setState(state)
            }
        });

        // subscribe to run info
        new ROSLIB.Topic({
            ros: this.state.ros,
            name: this.state.topic.run.path,
            messageType: this.state.topic.run.type
        }).subscribe(function (message) {
            if (message) {
                let state = self.state
                state['topic']['run']['value'] = {
                    runtime: message.runtime,
                    detection_started: message.detection_started
                }
                self.setState(state)
            }
        });

        // subscribe to gps readings
        new ROSLIB.Topic({
            ros: this.state.ros,
            name: this.state.topic.gps.path,
            messageType: this.state.topic.gps.type
        }).subscribe(function (message) {
            if (message) {
                let state = self.state
                state['topic']['gps']['value'] = {
                    fix: message.fix,
                    lat: message.lat,
                    lng: message.lng,
                    speed: message.speed
                }
                self.setState(state)
            }
        });

        // subscribe to crowd data
        new ROSLIB.Topic({
            ros: this.state.ros,
            name: this.state.topic.compute.path,
            messageType: this.state.topic.compute.type
        }).subscribe(function (message) {
            if (message) {
                let state = self.state
                state['topic']['compute']['value'] = {
                    people: message.people,
                    neck_breathers: message.neck_breathers,
                    density: message.density,
                    area: message.area
                }
                self.setState(state)
            }
        });

    }

    componentDidMount() {
        let self = this

        this.state.ros.on('connection', function () {
            console.log('Connected to websocket on ' + ROS_REMOTE.address + ':' + ROS_REMOTE.ws_port);
            self.initTopics()
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

            state['topic']['gps']['value']['fix'] = 0

            self.setState(state)
        });

    }


    render() {
        console.log(this.state.topic.compute)
        return (
            <RemoveScroll>
                <div>

                    <NavigationBar/>

                    <Row style={{
                        backgroundColor: '#eaeaea',
                        paddingTop: '10px',
                        paddingBottom: '10px'
                    }}>

                        <Col sm={{size: '2', offset: 0}}
                             style={{marginLeft: '40px'}}>
                            <Chip
                                icon={this.state.ros_connected
                                    ? <SignalWifi3BarIcon style={{color: '#ffffff'}}/>
                                    : <SignalWifiOffIcon style={{color: '#ffffff'}}/>}
                                style={{
                                    backgroundColor: this.state.ros_connected ? 'green' : 'red',
                                    color: 'white', fontSize: '16px'
                                }}
                                label={this.state.ros_connected
                                    ? 'ROS UP: ' + (this.state.topic.run.value.runtime / 60).toFixed(0) + ' min'
                                    : 'ROS DOWN'}
                            />
                        </Col>

                        <Col sm={{size: '2', offset: 0}}
                             style={{marginLeft: '130px'}}>
                            <Chip
                                icon={this.state.topic.run.value.detection_started
                                    ? <VideocamIcon style={{color: '#ffffff'}}/>
                                    : <VideocamOffIcon style={{color: '#ffffff'}}/>}
                                style={{
                                    backgroundColor: '#2082d9',
                                    color: 'white', fontSize: '16px'
                                }}
                                label={this.state.topic.run.value.detection_started
                                    ? 'DETECTION ON'
                                    : 'DETECTION OFF'}
                            />

                        </Col>

                        <Col sm={{size: '2', offset: 0}}
                             style={{marginLeft: '130px'}}>
                            <Chip
                                icon={this.state.topic.armed.value
                                    ? <ErrorSharpIcon style={{color: '#ffffff'}}/>
                                    : <CheckCircleSharpIcon style={{color: '#ffffff'}}/>}
                                style={{
                                    backgroundColor: this.state.topic.armed.value ? 'red' : 'green',
                                    color: 'white', fontSize: '16px'
                                }}
                                label={this.state.topic.armed.value ? 'ARMED' : 'DISARMED'}
                            />
                        </Col>

                        <Col sm={{size: '2', offset: 0}}
                             style={{marginLeft: '130px'}}>
                            <Chip
                                icon={this.state.topic.gps.value.fix > 0
                                    ? <GpsFixedIcon style={{color: '#ffffff'}}/>
                                    : <GpsOffIcon style={{color: '#ffffff'}}/>}
                                style={{
                                    backgroundColor: this.state.topic.gps.value.fix > 0 ? 'green' : '#e7ae00',
                                    color: 'white', fontSize: '16px'
                                }}
                                label={this.state.topic.gps.value.fix > 0
                                    ? 'GPS ON: ' + this.state.topic.gps.value.fix + ' sat'
                                    : 'GPS OFF'}
                            />
                        </Col>

                    </Row>

                    <Row style={{marginTop: '0px'}}>

                        <Col sm={{size: 0, offset: 0}} style={{width: '573px', marginLeft: '20px'}}>

                            <Row>
                                <Col>
                                    <Paper elevation={3}>

                                        <Card style={{
                                            marginTop: '10px',
                                            paddingLeft: '0px',
                                            paddingTop: '0px',
                                            paddingBottom: '0px'
                                        }}>
                                            <img
                                                style={{
                                                    backgroundColor: '#585858',
                                                    borderBottomLeftRadius: '3px',
                                                    borderBottomRightRadius: '3px'
                                                }}
                                                width={653}
                                                height={490}
                                                src={'http://' + ROS_REMOTE.address + ':' + ROS_REMOTE.video_port + '/' + ROS_REMOTE.video_path}
                                                alt={'Live Video Feed'}/>
                                        </Card>
                                    </Paper>
                                </Col>
                            </Row>

                        </Col>

                    </Row>

                </div>
            </RemoveScroll>
        );
    }
}

export default CrowdDataContainer