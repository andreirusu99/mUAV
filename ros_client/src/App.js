import './App.css';
import React from "react";
import DashboardContainer from "./page/dashboard/container/DashboardContainer"
import {BrowserRouter as Router, Route, Switch} from 'react-router-dom'
import 'fontsource-roboto';
import StatisticsContainer from "./page/statistics/container/StatisticsContainer";

class App extends React.Component {


    render() {

        return (
            <div style={{fontFamily: 'roboto'}}>
                <Router>
                    <div>
                        <Switch>

                            <Route exact path='/' render={() => <DashboardContainer/>}/>

                        </Switch>
                    </div>
                </Router>
            </div>
        )
    };
}

export default App