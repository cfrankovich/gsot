import React, { useState, useEffect } from "react";
import Sidebar from "./components/Sidebar";
import { createBrowserHistory } from "history";
import { BrowserRouter as Router, Route } from "react-router-dom";
import TopicView from "./components/TopicView";

const App: React.FC = () => {
    return (
        <>
            <Router>
                <Sidebar></Sidebar>
                <Route
                    path="/topic/:topic"
                    render={({ match }) => (
                        <TopicView topic={match.params.topic} />
                    )}
                />
                <Route
                    exact
                    path="/"
                    render={() => <TopicView topic={"overview"} />}
                />
            </Router>
        </>
    );
};

export default App;
