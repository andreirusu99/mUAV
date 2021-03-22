import './App.css';
import React from "react";
import DashboardContainer from "./page/dashboard/container/DashboardContainer"
import {BrowserRouter as Router, Route, Switch} from 'react-router-dom'
import 'fontsource-roboto';
import CrowdDataContainer from "./page/crowd_data/container/CrowdDataContainer";
import StatisticsContainer from "./page/statistics/container/StatisticsContainer";

class App extends React.Component {


    render() {

        return (
            <div style={{fontFamily: 'roboto'}}>
                <Router>
                    <div>
                        <Switch>

                            <Route exact path='/' render={() => <DashboardContainer/>}/>
                            <Route exact path='/dashboard' render={() => <DashboardContainer/>}/>
                            <Route exact path='/crowd_data' render={() => <CrowdDataContainer/>}/>
                            <Route exact path='/statistics' render={() => <StatisticsContainer/>}/>

                        </Switch>
                    </div>
                </Router>
            </div>
        )
    };
}

export default App