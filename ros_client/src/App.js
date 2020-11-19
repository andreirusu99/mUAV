import './App.css';
import React from "react";
import LiveContainer from "../src/page/live/container/LiveContainer"
import styles from "../src/common/style/main_style.css"
import {BrowserRouter as Router, Route, Switch} from 'react-router-dom'

class App extends React.Component {


  render() {

    return (
        <div className={styles.back}>
          <Router>
            <div>

              <Switch>

                <Route exact path='/live' render={() => <LiveContainer/>}/>

              </Switch>
            </div>
          </Router>
        </div>
    )
  };
}

export default App