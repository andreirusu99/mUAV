import React from 'react'
import logo from './common/image/drone-logo.png';

import {
    Nav,
    Navbar,
    NavbarBrand, Row, Col
} from 'reactstrap';
import {Button} from "@material-ui/core";
import DashboardSharpIcon from '@material-ui/icons/DashboardSharp';


const NavigationBar = () => (
    <div>
        <Navbar color="dark" light expand="md">

            <NavbarBrand href="/dashboard">
                <img src={logo}
                     style={{marginLeft: '30px'}}
                     width={"55"}
                     height={"38"}
                     alt="logo"
                />
            </NavbarBrand>

            <Nav className="mr-auto" navbar>
                <Row>
                    <Col sm={{size: '1', offset: '2'}}>

                        <Button
                            style={{color: '#30b2ff'}}
                            variant={"text"}
                            startIcon={<DashboardSharpIcon/>}
                            size={"large"}>
                            {"Dashboard"}
                        </Button>
                    </Col>
                </Row>
            </Nav>
        </Navbar>
    </div>
);

export default NavigationBar
