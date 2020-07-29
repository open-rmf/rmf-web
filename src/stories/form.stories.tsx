import React from 'react';

import AutoCompleteComponent from './BaseComponents/autocomplete';

export default { 
    title: 'Forms',
};

const states: boolean[] = [false, true];
const messages: string[] = ['', 'This is an error'];
const labels: string[] = ['Correct Inputs', 'Wrong Inputs'];

export const test = () => (
    <AutoCompleteComponent
        errorState={states}
        errorMessage={messages}
        labels={labels}
    />
)