# Copyright (c) Facebook, Inc. and its affiliates. All Rights Reserved
from .detr_vae import build as build_vae
from .detr_vae import build_comp as build_vae_comp
from .detr_vae import build_cnnmlp as build_cnnmlp

def build_ACT_model(args):
    return build_vae(args)

def build_CompACT_model(args):
    return build_vae_comp(args)

def build_CNNMLP_model(args):
    return build_cnnmlp(args)