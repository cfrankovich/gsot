import React from "react";
import { Link } from "react-router-dom";

type SideBarChildProps = {
    name: string;
};

const SidebarChild: React.FC<SideBarChildProps> = (props) => {
    return (
        <div className="sidebar-child">
            <Link to={`/topic${props.name}`}>{props.name}</Link>
        </div>
    );
};

export default SidebarChild;
