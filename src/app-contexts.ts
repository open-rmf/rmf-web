import { createContext } from "react";

/* Declares the UserContext which contains the currently logged in user, so
that any component may use if without having to pass the prop around */
export const UserContext = createContext({});