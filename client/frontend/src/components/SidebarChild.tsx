import React from "react";

type SideBarChildProps = {
    name: string;
};

const SidebarChild: React.FC<SideBarChildProps> = (props) => {
    return (
        <div className="sidebar-child">
            <p>{props.name}</p>
        </div>
    );
};

export default SidebarChild;
