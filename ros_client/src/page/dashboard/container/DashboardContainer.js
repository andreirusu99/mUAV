import React from 'react'
import {ROS_REMOTE} from '../../../common/host'
import {Badge, Card, CardHeader, Col, Progress, Row} from 'reactstrap';
import NavigationBar from '../../../navbar';
import ROSLIB from 'roslib'
import {CustomSlider} from '../component/CustomSlider'
import Chip from '@material-ui/core/Chip';
import ErrorSharpIcon from '@material-ui/icons/ErrorSharp';
import CheckCircleSharpIcon from '@material-ui/icons/CheckCircleSharp';
import PowerIcon from '@material-ui/icons/Power';
import BatteryChargingFullIcon from '@material-ui/icons/BatteryChargingFull';
import VideocamIcon from '@material-ui/icons/Videocam';
import PowerSettingsNewIcon from '@material-ui/icons/PowerSettingsNew';
import GamepadIcon from '@material-ui/icons/Gamepad';
import OpenWithIcon from '@material-ui/icons/OpenWith';
import SignalWifi3BarIcon from '@material-ui/icons/SignalWifi3Bar';
import SignalWifiOffIcon from '@material-ui/icons/SignalWifiOff';
import ExploreIcon from '@material-ui/icons/Explore';
import GpsOffIcon from '@material-ui/icons/GpsOff';
import GpsFixedIcon from '@material-ui/icons/GpsFixed';
import 'fontsource-roboto';


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
                    path: '/Control',
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
                        fixed: false
                    },
                    path: '/GPSReading',
                    // type: 'std_msgs/Float32'
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
            state['topic']['attitude']['value'] = {
                roll: 0,
                pitch: 0,
                yaw: 0,
                percentage: 0,
                power: 0,
                camera_angle: 0
            }
            self.setState(state)
        });

    }

    render() {
        return (
            <div>
                <NavigationBar/>

                <Row>

                    <Col sm={{size: '4', offset: 0}} style={{marginLeft: '40px', marginTop: '10px'}}>

                        <Row>
                            <Col className={'text-center'}>
                                <Chip
                                    icon={this.state.ros_connected
                                        ? <SignalWifi3BarIcon style={{color: '#ffffff'}}/>
                                        : <SignalWifiOffIcon style={{color: '#ffffff'}}/>}
                                    style={{
                                        backgroundColor: this.state.ros_connected ? 'green' : 'red',
                                        color: 'white', fontSize: '16px'
                                    }}
                                    label={this.state.ros_connected ? 'ROS connected' : 'ROS disconnected'}
                                />

                            </Col>
                        </Row>

                        <Row>
                            <Col>
                                <CardHeader className={'text-center'} style={{marginTop: '10px'}}>
                                    <VideocamIcon style={{color: '#de3636', float: 'left'}}/>
                                    <strong>{'Live Aerial Video Feed @ ' + this.state.topic.attitude.value.camera_angle + '°'}</strong>
                                </CardHeader>
                                <Card style={{paddingLeft: '5px', paddingTop: '5px', paddingBottom: '5px'}}>

                                    <img
                                        width={390}
                                        height={220}
                                        src={'http://' + ROS_REMOTE.address + ':' + ROS_REMOTE.video_port + '/' + ROS_REMOTE.video_path}
                                        alt={'Live Video Feed'}/>
                                </Card>
                            </Col>
                        </Row>

                        <Row>
                            <Col>

                                <CardHeader className={'text-center'} style={{marginTop: '10px'}}>
                                    <PowerSettingsNewIcon style={{float: 'left', color: '#3398e3'}}/>
                                    <strong>{'Power Management'}</strong>
                                </CardHeader>

                                <Card style={{padding: '15px', paddingRight: '25px'}}>

                                    <div style={{fontSize: '18px', marginTop: '5px'}}
                                         className={'text-center'}>

                                        <PowerIcon fontSize={"large"}
                                                   style={{float: 'left', color: '#d21010'}}/>
                                        <div style={{textAlign: 'left', marginBottom: '-25px'}}>
                                            {'Power'}
                                        </div>

                                        <strong>{this.state.topic.attitude.value.power + ' W'}</strong>
                                        <Progress animated
                                                  color={'danger'}
                                                  value={this.state.topic.attitude.value.power / 2}/>
                                    </div>

                                    <div style={{fontSize: '18px', marginTop: '20px'}}
                                         className={'text-center'}>
                                        <div style={{textAlign: 'left', marginBottom: '-25px'}}>
                                            <BatteryChargingFullIcon fontSize={"large"}
                                                                     style={{color: 'green', float: 'left'}}/>
                                            {'Battery'}
                                        </div>
                                        <strong>{this.state.topic.attitude.value.percentage + '%'}</strong>

                                        <Progress animated
                                                  color={'success'}
                                                  value={this.state.topic.attitude.value.percentage}/>
                                    </div>
                                </Card>
                            </Col>
                        </Row>

                    </Col>

                    <Col sm={{size: '3', offset: 0}} style={{marginTop: '10px'}}>

                        <Row>
                            <Col className={'text-center'}>
                                <Chip
                                    icon={this.state.topic.armed.value
                                        ? <ErrorSharpIcon style={{color: '#ffffff'}}/>
                                        : <CheckCircleSharpIcon style={{color: '#ffffff'}}/>}
                                    style={{
                                        backgroundColor: this.state.topic.armed.value ? 'red' : 'green',
                                        color: 'white', fontSize: '16px'
                                    }}
                                    label={this.state.topic.armed.value ? 'Armed - Watch Out!' : 'Disarmed - Safe!'}
                                />

                            </Col>
                        </Row>

                        <Row>

                            <Col>
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
                                        <strong>{Math.round(((this.state.topic.control.value.roll - 1000) * 200 / 1000 - 100)) + '%'}</strong>

                                        {this.state.topic.armed.value &&
                                        <strong>
                                            {Math.round(((this.state.topic.control.value.roll - 1000) * 200 / 1000 - 100)) > 0
                                                ? '→'
                                                : '←'}
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
                                        <strong>{Math.round(((this.state.topic.control.value.pitch - 1000) * 200 / 1000 - 100)) + '%'}
                                        </strong>

                                        {this.state.topic.armed.value &&
                                        <strong>
                                            {Math.round(((this.state.topic.control.value.pitch - 1000) * 200 / 1000 - 100)) > 0
                                                ? '↑'
                                                : '↓'}
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
                            </Col>
                        </Row>

                        <Row>

                            <Col>
                                <CardHeader className={'text-center'} style={{marginTop: '10px'}}>
                                    <OpenWithIcon style={{float: 'left', color: '#5ed416'}}/>
                                    <strong>{'UAV Orientation'}</strong>
                                </CardHeader>

                                <Card style={{
                                    padding: '15px',
                                    paddingBottom: '0px',
                                    fontSize: '16px',
                                    paddingLeft: '20px',
                                    paddingRight: '20px',
                                }}>

                                    <div className={'text-center'}
                                         style={{width: '100%', marginTop: '5px'}}>

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
                            </Col>
                        </Row>

                    </Col>

                    <Col sm={{size: '4', offset: 0}} style={{marginTop: '10px'}}>

                        <Row>
                            <Col className={'text-center'}>
                                <Chip
                                    icon={this.state.topic.gps.value.fixed
                                        ? <GpsFixedIcon style={{color: '#ffffff'}}/>
                                        : <GpsOffIcon style={{color: '#ffffff'}}/>}
                                    style={{
                                        backgroundColor: this.state.topic.gps.value.fixed ? 'green' : '#e7ae00',
                                        color: 'white', fontSize: '16px'
                                    }}
                                    label={this.state.topic.gps.value.fixed ? 'GPS Fixed' : 'GPS not fixed '}
                                />

                            </Col>
                        </Row>

                        {/*<Row>*/}

                        {/*    <Col>*/}
                        {/*        <CardHeader className={'text-center'} style={{marginTop: '10px'}}>*/}
                        {/*            <ExploreIcon style={{float: 'left', color: '#1f55a7'}}/>*/}
                        {/*            <strong>{'Surrounding Area Overview'}</strong>*/}
                        {/*        </CardHeader>*/}

                        {/*        <Card style={{padding: '15px', fontSize: '16px'}}>*/}


                        {/*        </Card>*/}
                        {/*    </Col>*/}
                        {/*</Row>*/}

                        {/*<Row>*/}

                        {/*    <Col>*/}
                        {/*        <CardHeader className={'text-center'} style={{marginTop: '10px'}}>*/}
                        {/*            <ExploreIcon style={{float: 'left', color: '#1f55a7'}}/>*/}
                        {/*            <strong>{'Location Information'}</strong>*/}
                        {/*        </CardHeader>*/}

                        {/*        <Card style={{padding: '15px', fontSize: '16px'}}>*/}


                        {/*        </Card>*/}
                        {/*    </Col>*/}
                        {/*</Row>*/}


                    </Col>

                </Row>


            </div>
        );
    }
}

export default DashboardContainer