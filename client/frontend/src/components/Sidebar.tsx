import React, { useState, useEffect } from "react";
import SidebarChild from "./SidebarChild";
import axios from "axios";

enum Status {
    NotConnected = "not connected",
    Connected = "connected",
    Initializing = "initializing",
    Checking = "checking connection",
    Error = "error",
}

function getStatusColor(status: Status): string {
    switch (status) {
        case Status.NotConnected:
            return "#f00";
        case Status.Connected:
            return "#0f0";
        case Status.Initializing:
            return "#ff0";
        case Status.Checking:
            return "#ff0";
        case Status.Error:
            return "#f00";
        default:
            return "#f0f";
    }
}

const Sidebar: React.FC = () => {
    const [status, setStatus] = useState<Status>(Status.NotConnected);
    const [topics, setTopics] = useState<string[]>([]);

    const requestAllTopics = () => {
        axios
            .get("/get-topics")
            .then((res) => {
                setStatus(Status.Initializing);
                const data: string = res.data.topics;
                const newTopics: string[] = data.split("\n");
                setTopics(newTopics);
                setStatus(Status.Connected);
            })
            .catch((err) => {
                setStatus(Status.Error);
                setTopics([]);
                console.error("Error fetching topics: ", err);
            });
    };

    // check connection every 10 seconds
    useEffect(() => {
        const checkConnectionStatus = () => {
            if (status === Status.Connected) {
                setStatus(Status.Checking);
                requestAllTopics();
            }
        };

        const interval = setInterval(checkConnectionStatus, 10000);

        return () => clearInterval(interval);
    }, [status]);

    return (
        <div className="sidebar">
            <div className="sidebar-child">
                <h3>GSOT (pre-release)</h3>

                <p>
                    status:{" "}
                    <span style={{ color: getStatusColor(status) }}>
                        {status}
                    </span>
                </p>

                {status === Status.NotConnected && (
                    <button onClick={requestAllTopics}>
                        intialize connection
                    </button>
                )}
                {status === Status.Error && (
                    <button onClick={requestAllTopics}>retry connection</button>
                )}
            </div>
            <div className="sidebar-child config-help-container">
                <a className="left">config</a>
                <a className="right">help</a>
            </div>

            {topics.map((str, index) => (
                <SidebarChild name={str}></SidebarChild>
            ))}
        </div>
    );
};

export default Sidebar;
