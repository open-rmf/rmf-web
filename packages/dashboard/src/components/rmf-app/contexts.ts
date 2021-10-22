import React from 'react';
import { RmfIngress } from './rmf-ingress';
import { RxRmf } from './rx-rmf';

export const RmfIngressContext = React.createContext<RmfIngress | undefined>(undefined);
export const RxRmfContext = React.createContext<RxRmf | null>(null);
