import {
  cartPickupPhaseInsertCartId,
  cartPickupPhaseInsertPickup,
  cartCustomPickupPhaseInsertCartId,
  cartCustomPickupPhaseInsertPickup,
  DeliveryCustomTaskDescription,
  deliveryPhaseInsertDropoff,
  deliveryPhaseInsertOnCancel,
  DeliveryPickupTaskDescription,
  DoubleComposeDeliveryTaskDescription,
  makeDefaultDeliveryCustomTaskDescription,
  makeDefaultDeliveryPickupTaskDescription,
  makeDefaultDoubleComposeDeliveryTaskDescription,
} from '.';

describe('Custom deliveries', () => {
  it('delivery pickup', () => {
    let deliveryPickupTaskDescription: DeliveryPickupTaskDescription | null = null;
    try {
      deliveryPickupTaskDescription = JSON.parse(`{
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
    } catch (e) {
      deliveryPickupTaskDescription = null;
    }
    expect(deliveryPickupTaskDescription).not.toEqual(null);

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
    expect(deliveryPickupTaskDescription).toEqual(description);
  });

  it('delivery_sequential_lot_pickup', () => {
    let deliveryCustomTaskDescription: DeliveryCustomTaskDescription | null = null;
    try {
      deliveryCustomTaskDescription = JSON.parse(`{
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
    } catch (e) {
      deliveryCustomTaskDescription = null;
    }
    expect(deliveryCustomTaskDescription).not.toEqual(null);

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
    expect(deliveryCustomTaskDescription).toEqual(description);
  });

  it('delivery_area_pickup', () => {
    let deliveryCustomTaskDescription: DeliveryCustomTaskDescription | null = null;
    try {
      deliveryCustomTaskDescription = JSON.parse(`{
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
    } catch (e) {
      deliveryCustomTaskDescription = null;
    }
    expect(deliveryCustomTaskDescription).not.toEqual(null);

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
    expect(deliveryCustomTaskDescription).toEqual(description);
  });

  it('double_compose_delivery', () => {
    let desc: DoubleComposeDeliveryTaskDescription | null = null;
    try {
      desc = JSON.parse(`{
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
                        "cart_id": "test_first_cart_id",
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
                        "cart_id": "test_second_cart_id",
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
    } catch (e) {
      desc = null;
    }
    expect(desc).not.toEqual(null);

    const description: DoubleComposeDeliveryTaskDescription =
      makeDefaultDoubleComposeDeliveryTaskDescription();
    let firstPickupPhase = cartPickupPhaseInsertPickup(
      description.phases[0],
      'test_first_pickup_place',
      'test_first_pickup_lot',
    );
    firstPickupPhase = cartPickupPhaseInsertCartId(firstPickupPhase, 'test_first_cart_id');
    let firstDeliveryPhase = deliveryPhaseInsertDropoff(
      description.phases[1],
      'test_first_dropoff_place',
    );
    firstDeliveryPhase = deliveryPhaseInsertOnCancel(firstDeliveryPhase, [
      'test_waypoint_1',
      'test_waypoint_2',
      'test_waypoint_3',
    ]);
    let secondPickupPhase = cartPickupPhaseInsertPickup(
      description.phases[3],
      'test_second_pickup_place',
      'test_second_pickup_lot',
    );
    secondPickupPhase = cartPickupPhaseInsertCartId(secondPickupPhase, 'test_second_cart_id');
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
    expect(desc).toEqual(description);
  });
});
