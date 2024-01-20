import { Request, Response } from "express";
import {
    startMetricDataTCPServer,
    sendConfigRequest,
    startConfigTCPServer,
    stopTCPServers,
} from "./tcpClients";
import {
    initializeLogger,
    getTopicData,
    stopLogger,
    getLoggerStatus,
} from "./logger";

const express = require("express");
const port = 8080;

const app = express();
app.use(express.json());

startConfigTCPServer();
startMetricDataTCPServer();

app.get("/get-topics", async (_req: Request, res: Response) => {
    try {
        const tcpRes = await sendConfigRequest("TRANSMIT_ALL_TOPICS");
        res.json({ topics: tcpRes });
    } catch (err) {
        console.error("Error getting topics: ", err);
        res.status(500).send("Failed to retrieve topics.");
    }
});

app.post("/update-log-file", async (req: Request, res: Response) => {
    try {
        await initializeLogger(req.body.logDirPrefix, req.body.topics);
        res.status(200).send("Logger initialized.");
    } catch (err) {
        console.error("Error initializing logger: ", err);
        res.status(500).send("Error initializing logger.");
    }
});

app.get("/get-topic-data", async (req: Request, res: Response) => {
    const topic = req.query.topic;
    if (typeof topic === "string") {
        try {
            const topicData = await getTopicData(topic);
            res.send(topicData);
        } catch (err) {
            res.status(500).send("Error reading topic data.");
        }
    } else {
        res.status(400).send("invalid topic parameter.");
    }
});

app.post("/stop-everything", async (_req: Request, res: Response) => {
    try {
        stopTCPServers();
    } catch (err) {
        res.status(500).send("Error stopping TCP servers.");
    }

    try {
        stopLogger();
    } catch (err) {
        res.status(500).send("Error stopping logger.");
    }

    res.status(200).send("Reset TCP servers and stopped logger.");
});

app.get("/logger-status", async (req: Request, res: Response) => {
    try {
        const ls: Boolean = getLoggerStatus();
        res.status(200).send(ls);
    } catch (err) {
        res.status(500).send("Error getting logger status.");
    }
});

app.listen(port, () => {
    console.log(`Server is listening on port ${port}`);
});
