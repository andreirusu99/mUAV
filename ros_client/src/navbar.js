import React from 'react'
import logo from './common/image/drone-logo.png';

import {
    Nav,
    Navbar,
    NavbarBrand, Row, Col, NavItem, NavLink
} from 'reactstrap';
import {Button} from "@material-ui/core";
import DashboardSharpIcon from '@material-ui/icons/DashboardSharp';
import BarChartSharpIcon from '@material-ui/icons/BarChartSharp';
import LayersIcon from '@material-ui/icons/Layers';


const NavigationBar = () => (
    <div>
        <Navbar color="dark" expand="md" style={{height: '65px'}}>

            <NavbarBrand href="/dashboard">
                <img src={logo}
                     style={{marginLeft: '30px'}}
                     width={"55"}
                     height={"38"}
                     alt="logo"
                />
            </NavbarBrand>

            <Nav className="mr-auto" navbar>

                <NavItem style={{marginLeft: '10px'}}>
                    <NavLink href={"/dashboard"}>
                        <Button
                            style={{color: '#30b2ff'}}
                            variant={"text"}
                            startIcon={<DashboardSharpIcon/>}
                            size={"large"}>
                            {"Dashboard"}
                        </Button>
                    </NavLink>
                </NavItem>

                <NavItem style={{marginLeft: '10px'}}>
                    <NavLink href={"/collection"}>
                        <Button
                            style={{color: '#ff6f00'}}
                            variant={"text"}
                            startIcon={<LayersIcon/>}
                            size={"large"}>
                            {"Collection"}
                        </Button>
                    </NavLink>
                </NavItem>

                <NavItem style={{marginLeft: '10px'}}>
                    <NavLink href={"/statistics"}>
                        <Button
                            style={{color: '#2ad433'}}
                            variant={"text"}
                            startIcon={<BarChartSharpIcon/>}
                            size={"large"}>
                            {"Statistics"}
                        </Button>
                    </NavLink>
                </NavItem>

            </Nav>
        </Navbar>
    </div>
);

export default NavigationBar
