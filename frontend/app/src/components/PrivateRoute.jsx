import React from "react";
import { Navigate } from "react-router-dom";
import { useRecoilValue } from "recoil";
import { authState } from "../recoil/authState";

const PrivateRoute = ({element}) => {
    const auth = useRecoilValue(authState);

    return auth.isAuthenticated ? element : <Navigate to="/login" />;
}

export default PrivateRoute;