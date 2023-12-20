import { Request, Response } from "express";
const express = require("express");
const app = express();
const port = 8080;

app.get("/hello-world", (_req: Request, res: Response) => {
  res.send("hello world");
});

app.listen(port, () => {
  console.log(`Server is listening on port ${port}`);
});
