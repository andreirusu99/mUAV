import React from 'react'
import {ROS_REMOTE} from '../../../common/host'
import {Card, CardHeader, Col, Progress, Row} from 'reactstrap';
import ROSLIB from 'roslib'
import {CustomSlider} from '../component/CustomSlider'
import Chip from '@material-ui/core/Chip';
import ErrorSharpIcon from '@material-ui/icons/ErrorSharp';
import CheckCircleSharpIcon from '@material-ui/icons/CheckCircleSharp';
import FlashOnIcon from '@material-ui/icons/FlashOn';
import BatteryChargingFullIcon from '@material-ui/icons/BatteryChargingFull';
import VideocamIcon from '@material-ui/icons/Videocam';
import GamepadIcon from '@material-ui/icons/Gamepad';
import OpenWithIcon from '@material-ui/icons/OpenWith';
import SignalWifi3BarIcon from '@material-ui/icons/SignalWifi3Bar';
import SignalWifiOffIcon from '@material-ui/icons/SignalWifiOff';
import GpsOffIcon from '@material-ui/icons/GpsOff';
import GpsFixedIcon from '@material-ui/icons/GpsFixed';
import Paper from "@material-ui/core/Paper";
import ExploreIcon from "@material-ui/icons/Explore"
import ArrowBackIcon from '@material-ui/icons/ArrowBack';
import ArrowForwardIcon from '@material-ui/icons/ArrowForward';
import ArrowDownwardIcon from '@material-ui/icons/ArrowDownward';
import ArrowUpwardIcon from '@material-ui/icons/ArrowUpward';
import GoogleMap from "../component/GoogleMap";
import CallMissedIcon from '@material-ui/icons/CallMissed';
import {RemoveScroll} from "react-remove-scroll";
import {Slider} from "@material-ui/core";
import VideocamOffIcon from '@material-ui/icons/VideocamOff';
import WarningIcon from '@material-ui/icons/Warning';

// the boundary where the sonar reading is replaced with the barometer reading
const BARO_THRESH = 300 //cm
class DashboardContainer extends React.Component {

    constructor(props, context) {
        super(props, context);

        this.state = {

            ros: new ROSLIB.Ros({url: 'ws://' + ROS_REMOTE.address + ':' + ROS_REMOTE.ws_port}),

            ros_connected: false,

            topic: {
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
                },
                detection: {
                    value: {
                        people_count: 0,
                        neck_breathers: 0,
                        density: 0.0,
                        area: 0.0
                    },
                    path: '/Compute',
                    type: 'drone/ComputeMsg'
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

        // subscribe to control input
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
                if (message.axis[2] < 1040 && self.state.topic.armed.value)
                    state['topic']['control']['value']['throttle'] = 1040

                self.setState(state)
            }
        });

        // subscribe to craft info
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

        // subscribe to sonar readings
        new ROSLIB.Topic({
            ros: this.state.ros,
            name: this.state.topic.sonar.path,
            messageType: this.state.topic.sonar.type
        }).subscribe(function (message) {
            if (message) {
                let state = self.state
                if (message.data > BARO_THRESH)
                    state['topic']['sonar']['value'] = BARO_THRESH
                else
                    state['topic']['sonar']['value'] = message.data
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

        // subscribe to barometer readings
        new ROSLIB.Topic({
            ros: this.state.ros,
            name: this.state.topic.bmp.path,
            messageType: this.state.topic.bmp.type
        }).subscribe(function (message) {
            if (message) {
                let state = self.state
                state['topic']['bmp']['value'] = {
                    rel_alt: message.relative,
                    abs_alt: message.absolute,
                    temp: message.temp
                }
                self.setState(state)
            }
        });

        // subscribe to detection results
        new ROSLIB.Topic({
            ros: this.state.ros,
            name: this.state.topic.detection.path,
            messageType: this.state.topic.detection.type
        }).subscribe(function (message) {
            if (message) {
                let state = self.state
                state['topic']['detection']['value'] = {
                    people_count: message.people_count,
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
            state['topic']['attitude']['value'] = {
                roll: 0,
                pitch: 0,
                yaw: 0,
                percentage: 0,
                power: 0,
                camera_angle: 0
            }
            state['topic']['control']['value'] = {
                roll: 1500,
                pitch: 1500,
                yaw: 1500,
                throttle: 1000
            }
            state['topic']['gps']['value']['fix'] = 0
            state['topic']['gps']['value']['speed'] = 0.0
            state['topic']['bmp']['value'] = {
                rel_alt: 0.0,
                abs_alt: 0.0,
                temp: 0.0
            }
            state['topic']['sonar']['value'] = 0.0
            state['topic']['detection']['value'] = {
                people_count: 0,
                neck_breathers: 0,
                density: 0.0,
                area: 0
            }
            self.setState(state)
        });
    }


    render() {
        return (
            <RemoveScroll>
                <div>

                    <Row
                        style={{
                            backgroundColor: '#343A40',
                            paddingTop: '10px',
                            paddingBottom: '10px'
                        }}>
                        <Col className={'text-center'}>

                            {this.state.topic.run.value.detection_started
                                ? <div>
                                    <strong style={{color: "#FFFFFF", fontSize: '18px'}} className={'text-center'}>
                                        {'Total people: ' + this.state.topic.detection.value.people_count}
                                    </strong>

                                    <strong style={{color: "#FFFFFF", marginLeft: '50px', fontSize: '18px'}}
                                            className={'text-center'}>
                                        {'Total area: ' + this.state.topic.detection.value.area.toFixed(0) + ' m2'}
                                    </strong>

                                    <strong style={{color: "#FFFFFF", marginLeft: '50px', fontSize: '18px'}}
                                            className={'text-center'}>
                                        {'Density: ' + this.state.topic.detection.value.density + ' people/10m2'}
                                    </strong>
                                    {this.state.topic.detection.value.density > 1 &&
                                    <WarningIcon style={{color: '#E7AE00', marginLeft: '5px'}}/>
                                    }

                                    <strong style={{color: "#d42f2f", marginLeft: '50px', fontSize: '18px'}}
                                            className={'text-center'}>
                                        {'Too close: ' + this.state.topic.detection.value.neck_breathers + ' people'}
                                    </strong>
                                </div>
                                : <i style={{color: "#FFFFFF", fontSize:'18px'}}>
                                    {'Start the detection to show quick statistics here'}
                                </i>
                            }


                        </Col>
                    </Row>

                    <Row style={{
                        backgroundColor: '#eaeaea',
                        paddingTop: '10px',
                        paddingBottom: '10px'
                    }}>

                        <Col sm={{size: '2', offset: 0}}
                             style={{marginLeft: '45px'}}>
                            <Chip
                                icon={this.state.ros_connected
                                    ? <SignalWifi3BarIcon style={{color: '#ffffff'}}/>
                                    : <SignalWifiOffIcon style={{color: '#ffffff'}}/>}
                                style={{
                                    backgroundColor: this.state.ros_connected ? 'green' : 'red',
                                    color: 'white', fontSize: '16px'
                                }}
                                label={this.state.ros_connected
                                    ? 'CONNECTED'
                                    : 'NO SIGNAL'}
                            />
                        </Col>

                        <Col sm={{size: '2', offset: 0}}
                             style={{marginLeft: '15px'}}>
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
                             style={{marginLeft: '60px'}}>
                            <Chip
                                icon={this.state.topic.armed.value
                                    ? <ErrorSharpIcon style={{color: '#ffffff'}}/>
                                    : <CheckCircleSharpIcon style={{color: '#ffffff'}}/>}
                                style={{
                                    backgroundColor: this.state.topic.armed.value ? 'red' : 'green',
                                    color: 'white', fontSize: '16px'
                                }}
                                label={this.state.topic.armed.value
                                    ? 'ARMED'
                                    : 'DISARMED'}
                            />
                        </Col>

                        <Col sm={{size: '2', offset: 0}}
                             style={{marginLeft: '170px'}}>
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

                        <Col sm={{size: '4', offset: 0}} style={{marginLeft: '40px'}}>

                            <Row>
                                <Col>
                                    <Paper elevation={3}>

                                        <CardHeader className={'text-center'} style={{marginTop: '10px'}}>
                                            <VideocamIcon style={{color: '#de3636', float: 'left'}}/>
                                            <strong>{'Live Video Feed @ ' + this.state.topic.attitude.value.camera_angle + '°'}</strong>
                                        </CardHeader>
                                        <Card style={{paddingLeft: '0px', paddingTop: '0px', paddingBottom: '0px'}}>
                                            <img
                                                style={{
                                                    backgroundColor: '#585858',
                                                    borderBottomLeftRadius: '3px',
                                                    borderBottomRightRadius: '3px'
                                                }}
                                                width={404}
                                                height={303}
                                                src={'http://' + ROS_REMOTE.address + ':' + ROS_REMOTE.video_port + '/' + ROS_REMOTE.video_path}
                                                alt={'Live Video Feed'}/>
                                        </Card>
                                    </Paper>
                                </Col>
                            </Row>

                            <Row>
                                <Col>
                                    <Paper elevation={3}>

                                        <Card style={{padding: '15px', paddingRight: '25px'}}>

                                            <div style={{fontSize: '18px', marginTop: '-2px'}}
                                                 className={'text-center'}>

                                                <FlashOnIcon fontSize={'large'}
                                                             style={{float: 'left', color: '#DC3443',}}/>
                                                <div style={{textAlign: 'left', marginBottom: '-25px'}}>
                                                    {'Energy usage'}
                                                </div>

                                                <strong>{this.state.topic.attitude.value.power + ' W'}</strong>
                                                <Progress animated
                                                          color={'danger'}
                                                          value={this.state.topic.attitude.value.power / 2}/>
                                            </div>

                                            <div style={{fontSize: '18px', marginTop: '10px', marginBottom: '5px'}}
                                                 className={'text-center'}>
                                                <div style={{textAlign: 'left', marginBottom: '-25px'}}>
                                                    <BatteryChargingFullIcon fontSize={'large'}
                                                                             style={{color: '#26A843', float: 'left'}}/>
                                                    {'Battery status'}
                                                </div>
                                                <strong>{this.state.topic.attitude.value.percentage + '%'}</strong>

                                                <Progress animated
                                                          color={'success'}
                                                          value={this.state.topic.attitude.value.percentage}/>
                                            </div>
                                        </Card>
                                    </Paper>
                                </Col>
                            </Row>

                        </Col>

                        <Col sm={{size: '3', offset: 0}}>

                            <Row>
                                <Col>
                                    <Paper elevation={3}>
                                        <CardHeader className={'text-center'} style={{marginTop: '10px'}}>
                                            <GamepadIcon style={{float: 'left', color: '#ff6d00'}}/>
                                            <strong>{'Control Axes'}</strong>
                                        </CardHeader>

                                        <Card style={{padding: '15px', fontSize: '16px'}}>

                                            <div className={'text-center'} style={{width: '100%', marginTop: '5px'}}>
                                                <div style={{textAlign: 'left', marginBottom: '-25px'}}>
                                                    {'Throttle'}
                                                </div>
                                                <strong>{Math.round(((this.state.topic.control.value.throttle - 1000) / 1000) * 100) + '%'}</strong>
                                                <Progress
                                                    color={'success'}
                                                    value={Math.round(((this.state.topic.control.value.throttle - 1000) / 1000) * 100)}>
                                                    {this.state.topic.control.value.throttle}
                                                </Progress>
                                            </div>

                                            <div className={'text-center'}
                                                 style={{width: '100%', marginTop: '15px'}}>

                                                <div style={{textAlign: 'left', marginBottom: '-25px'}}>
                                                    {'Roll'}
                                                </div>

                                                {Math.abs(Math.round(((this.state.topic.control.value.roll - 1000) * 200 / 1000 - 100))) >= 10 &&
                                                <strong>
                                                    {Math.round(((this.state.topic.control.value.roll - 1000) * 200 / 1000 - 100)) > 0
                                                        ? <ArrowForwardIcon fontSize={"small"}/>
                                                        : <ArrowBackIcon fontSize={"small"}/>}
                                                </strong>}

                                                <strong>
                                                    {Math.abs(Math.round(((this.state.topic.control.value.roll - 1000) * 200 / 1000 - 100))) >= 10
                                                        ? Math.abs(Math.round(((this.state.topic.control.value.roll - 1000) * 200 / 1000 - 100))) + '%'
                                                        : "0%"}
                                                </strong>

                                                {Math.abs(Math.round(((this.state.topic.control.value.roll - 1000) * 200 / 1000 - 100))) >= 10 &&
                                                <strong>
                                                    {Math.round(((this.state.topic.control.value.roll - 1000) * 200 / 1000 - 100)) > 0
                                                        ? <ArrowForwardIcon fontSize={"small"}/>
                                                        : <ArrowBackIcon fontSize={"small"}/>}
                                                </strong>}

                                                <Progress multi>

                                                    <Progress bar
                                                              color={'info'}
                                                              value={Math.round(((this.state.topic.control.value.roll - 1000) / 1000) * 100) - 1}>
                                                        {this.state.topic.control.value.roll - 1000}
                                                    </Progress>

                                                    <Progress bar color={'warning'} value={2}/>

                                                    <Progress bar
                                                              color={'info'}
                                                              value={100 - Math.round(((this.state.topic.control.value.roll - 1000) / 1000) * 100) - 1}>
                                                        {2000 - this.state.topic.control.value.roll}
                                                    </Progress>

                                                </Progress>

                                            </div>

                                            <div className={'text-center'}
                                                 style={{width: '100%', marginTop: '15px'}}>

                                                <div style={{textAlign: 'left', marginBottom: '-25px'}}>
                                                    {'Pitch'}
                                                </div>

                                                {Math.abs(Math.round(((this.state.topic.control.value.pitch - 1000) * 200 / 1000 - 100))) >= 10 &&
                                                <strong>
                                                    {Math.round(((this.state.topic.control.value.pitch - 1000) * 200 / 1000 - 100)) > 0
                                                        ? <ArrowUpwardIcon fontSize={"small"}/>
                                                        : <ArrowDownwardIcon fontSize={"small"}/>}
                                                </strong>}

                                                <strong>
                                                    {Math.abs(Math.round(((this.state.topic.control.value.pitch - 1000) * 200 / 1000 - 100))) >= 10
                                                        ? Math.abs(Math.round(((this.state.topic.control.value.pitch - 1000) * 200 / 1000 - 100))) + '%'
                                                        : "0%"}
                                                </strong>

                                                {Math.abs(Math.round(((this.state.topic.control.value.pitch - 1000) * 200 / 1000 - 100))) >= 10 &&
                                                <strong>
                                                    {Math.round(((this.state.topic.control.value.pitch - 1000) * 200 / 1000 - 100)) > 0
                                                        ? <ArrowUpwardIcon fontSize={"small"}/>
                                                        : <ArrowDownwardIcon fontSize={"small"}/>}
                                                </strong>}

                                                <Progress multi>

                                                    <Progress bar
                                                              color={'info'}
                                                              value={Math.round(((this.state.topic.control.value.pitch - 1000) / 1000) * 100) - 1}>
                                                        {this.state.topic.control.value.pitch - 1000}
                                                    </Progress>

                                                    <Progress bar color={'warning'} value={2}/>

                                                    <Progress bar
                                                              color={'info'}
                                                              value={100 - Math.round(((this.state.topic.control.value.pitch - 1000) / 1000) * 100) - 1}>
                                                        {2000 - this.state.topic.control.value.pitch}
                                                    </Progress>

                                                </Progress>
                                            </div>

                                        </Card>
                                    </Paper>

                                </Col>
                            </Row>

                            <Row>
                                <Col>
                                    <Paper elevation={3}>
                                        <CardHeader className={'text-center'} style={{marginTop: '10px'}}>
                                            <OpenWithIcon style={{float: 'left', color: '#5ed416'}}/>
                                            <strong>{'UAV Orientation'}</strong>
                                        </CardHeader>
                                        <Card style={{
                                            padding: '15px',
                                            paddingBottom: '0px',
                                            fontSize: '16px',
                                            paddingLeft: '25px',
                                            paddingRight: '25px',
                                        }}>

                                            <div className={'text-center'}
                                                 style={{width: '100%'}}>

                                                <strong>{this.state.topic.attitude.value.roll.toFixed(1) + '°'}</strong>
                                                {' Roll'}

                                                <CustomSlider
                                                    track={false}
                                                    marks={[
                                                        {value: 1, label: '20°'},
                                                        {value: 12.5, label: '15°'},
                                                        {value: 25, label: '10°'},
                                                        {value: 37.5, label: '5°'},
                                                        {value: 50, label: '0°'},
                                                        {value: 62.5, label: '5°'},
                                                        {value: 75, label: '10°'},
                                                        {value: 87.5, label: '15°'},
                                                        {value: 98, label: '20°'},
                                                    ]}
                                                    value={((this.state.topic.attitude.value.roll + 20) * 100 / 40)}
                                                />
                                            </div>

                                            <div className={'text-center'}
                                                 style={{width: '100%', marginTop: '5px'}}>

                                                <strong>{this.state.topic.attitude.value.pitch.toFixed(1) + '°'}</strong>
                                                {' Pitch'}

                                                <CustomSlider
                                                    track={false}
                                                    marks={[
                                                        {value: 1, label: '20°'},
                                                        {value: 12.5, label: '15°'},
                                                        {value: 25, label: '10°'},
                                                        {value: 37.5, label: '5°'},
                                                        {value: 50, label: '0°'},
                                                        {value: 62.5, label: '5°'},
                                                        {value: 75, label: '10°'},
                                                        {value: 87.5, label: '15°'},
                                                        {value: 98, label: '20°'},
                                                    ]}
                                                    value={((this.state.topic.attitude.value.pitch + 20) * 100 / 40)}
                                                />
                                            </div>
                                        </Card>
                                    </Paper>
                                </Col>
                            </Row>

                        </Col>

                        <Col sm={{size: '4', offset: 0}}>

                            <Row>
                                <Col>
                                    <Paper elevation={3}>

                                        <CardHeader className={'text-center'} style={{marginTop: '10px'}}>
                                            <ExploreIcon style={{float: 'left', color: '#1f55a7'}}/>
                                            <strong>{'Location and Surroundings'}</strong>
                                        </CardHeader>
                                        <Card style={{
                                            paddingBottom: '10px',
                                            fontSize: '16px',
                                            height: '100%'
                                        }}>
                                            <GoogleMap
                                                center={{
                                                    lat: this.state.topic.gps.value.lat,
                                                    lng: this.state.topic.gps.value.lng
                                                }}
                                                markers={
                                                    <CallMissedIcon
                                                        lat={this.state.topic.gps.value.lat}
                                                        lng={this.state.topic.gps.value.lng}
                                                        fontSize={"large"}
                                                        style={{color: "#00eaff"}}
                                                    />
                                                }
                                            />

                                            <div style={{
                                                marginTop: '5px',
                                                paddingLeft: '50px',
                                                paddingRight: '50px',
                                                fontSize: '18px'
                                            }}>

                                                <div>

                                                    <div style={{float: 'left'}}>
                                                        {"Altitude"}
                                                        <i>{" (barometric)"}</i>
                                                    </div>

                                                    <strong
                                                        style={{float: "right"}}>
                                                        {this.state.topic.bmp.value.abs_alt.toFixed(0) + " m"}
                                                    </strong>
                                                </div>

                                                <div style={{marginTop: '35px'}}>

                                                    <div style={{float: 'left'}}>
                                                        {"Height"}
                                                        <i>{" (relative)"}</i>
                                                    </div>

                                                    <strong
                                                        style={{float: "right"}}>
                                                        {this.state.topic.armed.value
                                                            ? (this.state.topic.bmp.value.rel_alt).toFixed(1) + " m"
                                                            : 'GROUNDED'
                                                        }
                                                    </strong>
                                                </div>

                                                <div style={{marginTop: '70px'}}>

                                                    <div style={{float: 'left'}}>
                                                        {"Air Temperature"}
                                                    </div>

                                                    <strong
                                                        style={{float: "right"}}>
                                                        {(this.state.topic.bmp.value.temp).toFixed(1) + " °C"}
                                                    </strong>
                                                </div>

                                                <Slider
                                                    draggable={"false"}
                                                    style={{marginBottom: '0px', marginTop: '-10px', color: '#e33c3c'}}
                                                    marks={[
                                                        {value: 0, label: '0°'},
                                                        {value: 12.5, label: '5°'},
                                                        {value: 25, label: '10°'},
                                                        {value: 37.5, label: '15°'},
                                                        {value: 50, label: '20°'},
                                                        {value: 62.5, label: '25°'},
                                                        {value: 75, label: '30°'},
                                                        {value: 87.5, label: '35°'},
                                                        {value: 100, label: '40°'},
                                                    ]}
                                                    value={this.state.topic.bmp.value.temp.toFixed(1) * 100 / 40}
                                                />

                                            </div>


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

export default DashboardContainer