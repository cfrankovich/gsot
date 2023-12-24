import { Request, Response } from "express";
import {
    startMetricDataTCPServer,
    sendConfigRequest,
    startConfigTCPServer,
} from "./tcpClients";

const express = require("express");
const app = express();
const port = 8080;

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

app.listen(port, () => {
    console.log(`Server is listening on port ${port}`);
});
