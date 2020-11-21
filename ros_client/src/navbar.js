import React from 'react'
import logo from './common/image/drone-logo.png';

import {
    Nav,
    Navbar,
    NavbarBrand, Row, Col
} from 'reactstrap';
import {Button} from "@material-ui/core";

const textStyle = {
    color: 'white',
    textDecoration: 'none'
};


const NavigationBar = () => (
    <div>
        <Navbar color="dark" light expand="md">

            <NavbarBrand href="/dashboard">
                <img src={logo}
                     width={"40"}
                     height={"32"}
                     alt="logo"
                />
            </NavbarBrand>

            <Nav className="mr-auto" navbar>
                <Row>
                    <Col sm={{size: '1', offset: '2'}}>

                        <Button
                            size={"small"}
                            variant={"contained"}
                            color={"primary"}>
                            {"Dashboard"}
                        </Button>
                    </Col>
                </Row>
            </Nav>
        </Navbar>
    </div>
);

export default NavigationBar
