import { createContext } from "react";
import { ResourceConfigurationsType } from "./resource-manager";

/* Declares the IconContext which contains the icons used on the app*/
export const IconContext = createContext<ResourceConfigurationsType>({});
