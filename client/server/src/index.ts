import { Request, Response } from "express";
import {
    startMetricDataTCPServer,
    sendConfigRequest,
    startConfigTCPServer,
} from "./tcpClients";
import { initializeLogger } from "./logger";

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
        res.status(200);
    } catch (err) {
        console.error("Error initializing logger: ", err);
        res.status(500).send("Error initializing logger.");
    }
});

app.listen(port, () => {
    console.log(`Server is listening on port ${port}`);
});
