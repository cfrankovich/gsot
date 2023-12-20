import React, { useState, useEffect } from "react";
import axios from "axios";

const App: React.FC = () => {
  const [text, setText] = useState("");

  useEffect(() => {
    axios.get("/hello-world").then((res) => {
      setText(res.data);
    });
  });

  return <h1>{text}</h1>;
};

export default App;
