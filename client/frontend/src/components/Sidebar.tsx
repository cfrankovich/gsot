import React, { useState, useEffect, useRef } from "react";
import SidebarChild from "./SidebarChild";
import axios from "axios";
import { Link } from "react-router-dom";
import { useHistory } from "react-router-dom";

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
    const [loggerStatus, setLoggerStatus] = useState<boolean>(false);

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
                resetEverything();
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

    const logDirPrefixInputRef = useRef<HTMLInputElement>(null);

    const history = useHistory();
    const resetEverything = () => {
        axios
            .post("/stop-everything")
            .then((_res) => {
                setTopics([]);
                setStatus(Status.NotConnected);
                setLoggerStatus(false);
                history.push("/");
            })
            .catch((err) => {
                alert("Stop failed: " + err);
            });
    };

    const startLogger = () => {
        if (logDirPrefixInputRef.current == null) {
            return;
        }
        if (status !== Status.Connected) {
            alert("Status is offline.");
            return;
        }

        axios
            .post("/update-log-file", {
                logDirPrefix: logDirPrefixInputRef.current.value,
                topics: topics,
            })
            .then((res) => {
                setLoggerStatus(true);
            })
            .catch((_err) => {
                setLoggerStatus(false);
                alert("Failed to update log file.");
            });
        logDirPrefixInputRef.current.value = "";
    };

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

                {!loggerStatus && (
                    <>
                        <p>
                            logger status:{" "}
                            <span style={{ color: "#f00" }}>offline</span>
                        </p>
                        <button onClick={startLogger}>initialize logger</button>
                    </>
                )}
                {loggerStatus && (
                    <>
                        <p>
                            logger status:{" "}
                            <span style={{ color: "#0f0" }}>online</span>
                        </p>

                        <button onClick={startLogger}>restart logger</button>
                    </>
                )}

                <input
                    placeholder="log dir prefix (default is date)"
                    ref={logDirPrefixInputRef}
                ></input>

                <br></br>
                <button onClick={resetEverything}>stop</button>
            </div>

            <div className="sidebar-child config-help-container">
                <a className="left">config</a>
                <a className="right">logs</a>
            </div>

            {status === Status.Connected && (
                <div className="sidebar-child">
                    <Link to={`/`}>/*</Link>
                </div>
            )}

            {topics.map((str, index) => (
                <SidebarChild name={str}></SidebarChild>
            ))}
        </div>
    );
};

export default Sidebar;
