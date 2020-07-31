import { createContext } from "react";
import { IconConfigurationsType } from "./icons-manager";

/* Declares the IconContext which contains the icons used on the app*/
export const IconContext = createContext<IconConfigurationsType>({});
