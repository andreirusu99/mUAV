import './App.css';
import React from "react";
import DashboardContainer from "./page/dashboard/container/DashboardContainer"
import styles from "../src/common/style/main_style.css"
import {BrowserRouter as Router, Route, Switch} from 'react-router-dom'
import 'fontsource-roboto';

class App extends React.Component {


    render() {

        return (
            <div style={{fontFamily: 'roboto'}}>
                <Router>
                    <div>

                        <Switch>

                            <Route exact path='/dashboard' render={() => <DashboardContainer/>}/>

                        </Switch>
                    </div>
                </Router>
            </div>
        )
    };
}

export default App