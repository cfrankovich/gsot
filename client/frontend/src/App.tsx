import React, { useState, useEffect } from "react";
import axios from "axios";

const App: React.FC = () => {
    const [topics, setTopics] = useState<string[]>([]);

    const request_all_topics = () => {
        axios
            .get("/get-topics")
            .then((res) => {
                const data: string = res.data.topics;
                const newTopics: string[] = data.split("\n");
                setTopics(newTopics);
            })
            .catch((err) => {
                console.error("Error fetching topics: ", err);
            });
    };

    return (
        <>
            <h1>gsot</h1>
            <button onClick={request_all_topics}>get topics</button>
            <p>Data:</p>
            <ul>
                {topics.map((str, index) => (
                    <li key={index}>{str}</li>
                ))}
            </ul>
        </>
    );
};

export default App;
