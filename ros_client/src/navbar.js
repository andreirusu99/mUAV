import React from 'react'
import logo from './common/image/logo512.png';

import {
    DropdownItem,
    DropdownMenu,
    DropdownToggle,
    Nav,
    Navbar,
    NavbarBrand,
    NavLink,
    UncontrolledDropdown
} from 'reactstrap';

const textStyle = {
    color: 'white',
    textDecoration: 'none'
};


const NavigationBar = () => (
    <div>
        <Navbar color="dark" light expand="md">
            <NavbarBrand href="/live">
                <img src={logo} width={"32"}
                     height={"32"}
                     alt="logo"
                />
            </NavbarBrand>
            <Nav className="mr-auto" navbar>

            </Nav>
        </Navbar>
    </div>
);

export default NavigationBar
