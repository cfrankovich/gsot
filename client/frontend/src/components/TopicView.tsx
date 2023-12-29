import React from "react";

type TopicViewProps = {
    topic: string;
};

const TopicView: React.FC<TopicViewProps> = (props) => {
    return (
        <div className="viewer">
            <p>{props.topic}</p>
        </div>
    );
};

export default TopicView;
