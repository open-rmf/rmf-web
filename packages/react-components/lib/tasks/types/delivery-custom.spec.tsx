import {
  cartCustomPickupPhaseInsertCartId,
  cartCustomPickupPhaseInsertPickup,
  CartPickupPhase,
  cartPickupPhaseInsertCartId,
  cartPickupPhaseInsertPickup,
  DeliveryCustomTaskDescription,
  deliveryPhaseInsertDropoff,
  deliveryPhaseInsertOnCancel,
  DeliveryPickupTaskDescription,
  DeliveryWithCancellationPhase,
  DoubleComposeDeliveryTaskDescription,
  makeDefaultDeliveryCustomTaskDescription,
  makeDefaultDeliveryPickupTaskDescription,
  makeDefaultDoubleComposeDeliveryTaskDescription,
} from '.';

describe('Custom deliveries', () => {
  it('create valid pickup phase', () => {
    const parsedPhase: CartPickupPhase = JSON.parse(`{
      "activity": {
        "category": "sequence",
        "description": {
          "activities": [
            {
              "category": "go_to_place",
              "description": "test_pickup_place"
            },
            {
              "category": "perform_action",
              "description": {
                "unix_millis_action_duration_estimate": 60000,
                "category": "delivery_pickup",
                "description": {
                  "cart_id": "test_cart_id",
                  "pickup_lot": "test_pickup_lot"
                }
              }
            }
          ]
        }
      }
    }
    `) as CartPickupPhase;

    const defaultDesc = makeDefaultDeliveryPickupTaskDescription();
    let insertedPhase = cartPickupPhaseInsertPickup(
      defaultDesc.phases[0],
      'test_pickup_place',
      'test_pickup_lot',
    );
    insertedPhase = cartPickupPhaseInsertCartId(insertedPhase, 'test_cart_id');
    expect(parsedPhase).toEqual(insertedPhase);
  });

  it('create valid delivery with cancellation phase', () => {
    const parsedPhase: DeliveryWithCancellationPhase = JSON.parse(`{
      "activity": {
        "category": "sequence",
        "description": {
          "activities": [
            {
              "category": "go_to_place",
              "description": "test_dropoff_place"
            }
          ]
        }
      },
      "on_cancel": [
        {
          "category": "sequence",
          "description": [
            {
              "category": "go_to_place",
              "description": {
                "one_of": [
                  {
                    "waypoint": "test_waypoint_1"
                  },
                  {
                    "waypoint": "test_waypoint_2"
                  },
                  {
                    "waypoint": "test_waypoint_3"
                  }
                ],
                "constraints": [
                  {
                    "category": "prefer_same_map",
                    "description": ""
                  }
                ]
              }
            },
            {
              "category": "perform_action",
              "description": {
                "unix_millis_action_duration_estimate": 60000,
                "category": "delivery_dropoff",
                "description": {}
              }
            }
          ]
        }
      ]
    }
    `) as DeliveryWithCancellationPhase;

    const defaultDesc = makeDefaultDeliveryPickupTaskDescription();
    let insertedPhase = deliveryPhaseInsertDropoff(defaultDesc.phases[1], 'test_dropoff_place');
    insertedPhase = deliveryPhaseInsertOnCancel(insertedPhase, [
      'test_waypoint_1',
      'test_waypoint_2',
      'test_waypoint_3',
    ]);
    expect(parsedPhase).toEqual(insertedPhase);
  });

  it('create valid delivery pickup task description', () => {
    const parsedDescription: DeliveryPickupTaskDescription = JSON.parse(`{
      "category": "delivery_pickup",
      "phases": [
        {
          "activity": {
            "category": "sequence",
            "description": {
              "activities": [
                {
                  "category": "go_to_place",
                  "description": "test_pickup_place"
                },
                {
                  "category": "perform_action",
                  "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "delivery_pickup",
                    "description": {
                      "cart_id": "test_cart_id",
                      "pickup_lot": "test_pickup_lot"
                    }
                  }
                }
              ]
            }
          }
        },
        {
          "activity": {
            "category": "sequence",
            "description": {
              "activities": [
                {
                  "category": "go_to_place",
                  "description": "test_dropoff_place"
                }
              ]
            }
          },
          "on_cancel": [
            {
              "category": "sequence",
              "description": [
                {
                  "category": "go_to_place",
                  "description": {
                    "one_of": [
                      {
                        "waypoint": "test_waypoint_1"
                      },
                      {
                        "waypoint": "test_waypoint_2"
                      },
                      {
                        "waypoint": "test_waypoint_3"
                      }
                    ],
                    "constraints": [
                      {
                        "category": "prefer_same_map",
                        "description": ""
                      }
                    ]
                  }
                },
                {
                  "category": "perform_action",
                  "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "delivery_dropoff",
                    "description": {}
                  }
                }
              ]
            }
          ]
        },
        {
          "activity": {
            "category": "sequence",
            "description": {
              "activities": [
                {
                  "category": "perform_action",
                  "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "delivery_dropoff",
                    "description": {}
                  }
                }
              ]
            }
          }
        }
      ]
    }
    `) as DeliveryPickupTaskDescription;

    const description = makeDefaultDeliveryPickupTaskDescription();
    let pickupPhase = cartPickupPhaseInsertPickup(
      description.phases[0],
      'test_pickup_place',
      'test_pickup_lot',
    );
    pickupPhase = cartPickupPhaseInsertCartId(pickupPhase, 'test_cart_id');
    let deliveryPhase = deliveryPhaseInsertDropoff(description.phases[1], 'test_dropoff_place');
    deliveryPhase = deliveryPhaseInsertOnCancel(deliveryPhase, [
      'test_waypoint_1',
      'test_waypoint_2',
      'test_waypoint_3',
    ]);
    description.phases[0] = pickupPhase;
    description.phases[1] = deliveryPhase;
    expect(parsedDescription).toEqual(description);
  });

  it('create valid delivery_sequential_lot_pickup task description', () => {
    const parsedDescription: DeliveryCustomTaskDescription = JSON.parse(`{
      "category": "delivery_sequential_lot_pickup",
      "phases": [
        {
          "activity": {
            "category": "sequence",
            "description": {
              "activities": [
                {
                  "category": "go_to_place",
                  "description": "test_pickup_place"
                },
                {
                  "category": "perform_action",
                  "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "delivery_sequential_lot_pickup",
                    "description": {
                      "cart_id": "test_cart_id",
                      "pickup_zone": "test_pickup_zone"
                    }
                  }
                }
              ]
            }
          }
        },
        {
          "activity": {
            "category": "sequence",
            "description": {
              "activities": [
                {
                  "category": "go_to_place",
                  "description": "test_dropoff_place"
                }
              ]
            }
          },
          "on_cancel": [
            {
              "category": "sequence",
              "description": [
                {
                  "category": "go_to_place",
                  "description": {
                    "one_of": [
                      {
                        "waypoint": "test_waypoint_1"
                      },
                      {
                        "waypoint": "test_waypoint_2"
                      },
                      {
                        "waypoint": "test_waypoint_3"
                      }
                    ],
                    "constraints": [
                      {
                        "category": "prefer_same_map",
                        "description": ""
                      }
                    ]
                  }
                },
                {
                  "category": "perform_action",
                  "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "delivery_dropoff",
                    "description": {}
                  }
                }
              ]
            }
          ]
        },
        {
          "activity": {
            "category": "sequence",
            "description": {
              "activities": [
                {
                  "category": "perform_action",
                  "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "delivery_dropoff",
                    "description": {}
                  }
                }
              ]
            }
          }
        }
      ]
    }
    `) as DeliveryCustomTaskDescription;

    const description: DeliveryCustomTaskDescription = makeDefaultDeliveryCustomTaskDescription(
      'delivery_sequential_lot_pickup',
    );
    let pickupPhase = cartCustomPickupPhaseInsertPickup(
      description.phases[0],
      'test_pickup_place',
      'test_pickup_zone',
    );
    pickupPhase = cartCustomPickupPhaseInsertCartId(pickupPhase, 'test_cart_id');
    let deliveryPhase = deliveryPhaseInsertDropoff(description.phases[1], 'test_dropoff_place');
    deliveryPhase = deliveryPhaseInsertOnCancel(deliveryPhase, [
      'test_waypoint_1',
      'test_waypoint_2',
      'test_waypoint_3',
    ]);
    description.phases[0] = pickupPhase;
    description.phases[1] = deliveryPhase;
    expect(parsedDescription).toEqual(description);
  });

  it('create valid delivery_area_pickup task description', () => {
    const parsedDescription: DeliveryCustomTaskDescription = JSON.parse(`{
      "category": "delivery_area_pickup",
      "phases": [
        {
          "activity": {
            "category": "sequence",
            "description": {
              "activities": [
                {
                  "category": "go_to_place",
                  "description": "test_pickup_place"
                },
                {
                  "category": "perform_action",
                  "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "delivery_area_pickup",
                    "description": {
                      "cart_id": "test_cart_id",
                      "pickup_zone": "test_pickup_zone"
                    }
                  }
                }
              ]
            }
          }
        },
        {
          "activity": {
            "category": "sequence",
            "description": {
              "activities": [
                {
                  "category": "go_to_place",
                  "description": "test_dropoff_place"
                }
              ]
            }
          },
          "on_cancel": [
            {
              "category": "sequence",
              "description": [
                {
                  "category": "go_to_place",
                  "description": {
                    "one_of": [
                      {
                        "waypoint": "test_waypoint_1"
                      },
                      {
                        "waypoint": "test_waypoint_2"
                      },
                      {
                        "waypoint": "test_waypoint_3"
                      }
                    ],
                    "constraints": [
                      {
                        "category": "prefer_same_map",
                        "description": ""
                      }
                    ]
                  }
                },
                {
                  "category": "perform_action",
                  "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "delivery_dropoff",
                    "description": {}
                  }
                }
              ]
            }
          ]
        },
        {
          "activity": {
            "category": "sequence",
            "description": {
              "activities": [
                {
                  "category": "perform_action",
                  "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "delivery_dropoff",
                    "description": {}
                  }
                }
              ]
            }
          }
        }
      ]
    }
    `) as DeliveryCustomTaskDescription;

    const description: DeliveryCustomTaskDescription =
      makeDefaultDeliveryCustomTaskDescription('delivery_area_pickup');
    let pickupPhase = cartCustomPickupPhaseInsertPickup(
      description.phases[0],
      'test_pickup_place',
      'test_pickup_zone',
    );
    pickupPhase = cartCustomPickupPhaseInsertCartId(pickupPhase, 'test_cart_id');
    let deliveryPhase = deliveryPhaseInsertDropoff(description.phases[1], 'test_dropoff_place');
    deliveryPhase = deliveryPhaseInsertOnCancel(deliveryPhase, [
      'test_waypoint_1',
      'test_waypoint_2',
      'test_waypoint_3',
    ]);
    description.phases[0] = pickupPhase;
    description.phases[1] = deliveryPhase;
    expect(parsedDescription).toEqual(description);
  });

  it('create valid double_compose_delivery task description', () => {
    const parsedDescription: DoubleComposeDeliveryTaskDescription = JSON.parse(`{
      "category": "delivery_pickup",
      "phases": [
        {
          "activity": {
            "category": "sequence",
            "description": {
              "activities": [
                {
                  "category": "go_to_place",
                  "description": "test_first_pickup_place"
                },
                {
                  "category": "perform_action",
                  "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "delivery_pickup",
                    "description": {
                      "cart_id": "",
                      "pickup_lot": "test_first_pickup_lot"
                    }
                  }
                }
              ]
            }
          }
        },
        {
          "activity": {
            "category": "sequence",
            "description": {
              "activities": [
                {
                  "category": "go_to_place",
                  "description": "test_first_dropoff_place"
                }
              ]
            }
          },
          "on_cancel": [
            {
              "category": "sequence",
              "description": [
                {
                  "category": "go_to_place",
                  "description": {
                    "one_of": [
                      {
                        "waypoint": "test_waypoint_1"
                      },
                      {
                        "waypoint": "test_waypoint_2"
                      },
                      {
                        "waypoint": "test_waypoint_3"
                      }
                    ],
                    "constraints": [
                      {
                        "category": "prefer_same_map",
                        "description": ""
                      }
                    ]
                  }
                },
                {
                  "category": "perform_action",
                  "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "delivery_dropoff",
                    "description": {}
                  }
                }
              ]
            }
          ]
        },
        {
          "activity": {
            "category": "sequence",
            "description": {
              "activities": [
                {
                  "category": "perform_action",
                  "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "delivery_dropoff",
                    "description": {}
                  }
                }
              ]
            }
          }
        },
        {
          "activity": {
            "category": "sequence",
            "description": {
              "activities": [
                {
                  "category": "go_to_place",
                  "description": "test_second_pickup_place"
                },
                {
                  "category": "perform_action",
                  "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "delivery_pickup",
                    "description": {
                      "cart_id": "",
                      "pickup_lot": "test_second_pickup_lot"
                    }
                  }
                }
              ]
            }
          }
        },
        {
          "activity": {
            "category": "sequence",
            "description": {
              "activities": [
                {
                  "category": "go_to_place",
                  "description": "test_second_dropoff_place"
                }
              ]
            }
          },
          "on_cancel": [
            {
              "category": "sequence",
              "description": [
                {
                  "category": "go_to_place",
                  "description": {
                    "one_of": [
                      {
                        "waypoint": "test_waypoint_1"
                      },
                      {
                        "waypoint": "test_waypoint_2"
                      },
                      {
                        "waypoint": "test_waypoint_3"
                      }
                    ],
                    "constraints": [
                      {
                        "category": "prefer_same_map",
                        "description": ""
                      }
                    ]
                  }
                },
                {
                  "category": "perform_action",
                  "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "delivery_dropoff",
                    "description": {}
                  }
                }
              ]
            }
          ]
        },
        {
          "activity": {
            "category": "sequence",
            "description": {
              "activities": [
                {
                  "category": "perform_action",
                  "description": {
                    "unix_millis_action_duration_estimate": 60000,
                    "category": "delivery_dropoff",
                    "description": {}
                  }
                }
              ]
            }
          }
        }
      ]
    }
    `) as DoubleComposeDeliveryTaskDescription;

    const description: DoubleComposeDeliveryTaskDescription =
      makeDefaultDoubleComposeDeliveryTaskDescription();
    const firstPickupPhase = cartPickupPhaseInsertPickup(
      description.phases[0],
      'test_first_pickup_place',
      'test_first_pickup_lot',
    );
    let firstDeliveryPhase = deliveryPhaseInsertDropoff(
      description.phases[1],
      'test_first_dropoff_place',
    );
    firstDeliveryPhase = deliveryPhaseInsertOnCancel(firstDeliveryPhase, [
      'test_waypoint_1',
      'test_waypoint_2',
      'test_waypoint_3',
    ]);
    const secondPickupPhase = cartPickupPhaseInsertPickup(
      description.phases[3],
      'test_second_pickup_place',
      'test_second_pickup_lot',
    );
    let secondDeliveryPhase = deliveryPhaseInsertDropoff(
      description.phases[4],
      'test_second_dropoff_place',
    );
    secondDeliveryPhase = deliveryPhaseInsertOnCancel(secondDeliveryPhase, [
      'test_waypoint_1',
      'test_waypoint_2',
      'test_waypoint_3',
    ]);
    description.phases[0] = firstPickupPhase;
    description.phases[1] = firstDeliveryPhase;
    description.phases[3] = secondPickupPhase;
    description.phases[4] = secondDeliveryPhase;
    expect(parsedDescription).toEqual(description);
  });
});
