import React from 'react'
import {ROS_REMOTE} from "../../../common/host"
import {Card, CardHeader, Col, Row} from "reactstrap";
import NavigationBar from "../../../navbar";

class LiveContainer extends React.Component {

    constructor(props, context) {
        super(props, context);
        this.state = {}
    }

    render() {
        return (
            <div>
                <NavigationBar/>
                <Col sm={{size: '4', offset: 0}} style={{marginLeft: "10px"}}>

                    <CardHeader style={{marginTop: "10px"}}>
                        <strong>Live Aerial Video Feed</strong>
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