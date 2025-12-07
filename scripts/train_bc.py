"""
Script to train the ACC policy using Behavior Cloning (BC) from collected expert data.
"""

import h5py
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset
import argparse
from rsl_rl.modules import ActorCritic

def train_bc(args):
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")

    # 1. Load Data
    print(f"Loading data from {args.dataset}...")
    with h5py.File(args.dataset, 'r') as f:
        obs = torch.from_numpy(f['obs'][:]).float()
        actions = torch.from_numpy(f['actions'][:]).float()
    
    print(f"Dataset shape: Obs {obs.shape}, Actions {actions.shape}")

    # 2. Compute Statistics for Normalization
    print("Computing observation statistics...")
    obs_mean = obs.mean(dim=0)
    obs_std = obs.std(dim=0)
    # Handle zero std (constant features)
    obs_std[obs_std < 1e-6] = 1.0
    
    # 3. Prepare DataLoader
    dataset = TensorDataset(obs, actions)
    # Split train/val
    train_size = int(0.9 * len(dataset))
    val_size = len(dataset) - train_size
    train_set, val_set = torch.utils.data.random_split(dataset, [train_size, val_size])
    
    train_loader = DataLoader(train_set, batch_size=args.batch_size, shuffle=True)
    val_loader = DataLoader(val_set, batch_size=args.batch_size, shuffle=False)

    # 4. Initialize Model using RSL-RL ActorCritic
    print("Initializing RSL-RL ActorCritic model...")
    # We need to mock the 'obs' dict structure that ActorCritic expects for initialization
    # It uses the shape of tensors in this dict to determine input dimensions.
    # ACC config: 75 obs
    # Note: ActorCritic expects obs to be a TensorDict or dict
    # We create a dummy dict with 'policy' key as expected by default config
    mock_obs = {"policy": torch.zeros(1, 75, device=device)}
    
    # We need to define obs_groups to tell it which keys to use
    obs_groups = {"policy": ["policy"], "critic": ["policy"]} # Critic uses same obs for now (or privileged if we had it)
    
    model = ActorCritic(
        obs=mock_obs,
        obs_groups=obs_groups,
        num_actions=18,
        actor_hidden_dims=[128, 128, 128],
        critic_hidden_dims=[128, 128, 128],
        activation="elu",
        init_noise_std=1.0,
        actor_obs_normalization=True,
        critic_obs_normalization=True,
    )
    model.to(device)
    
    # Set normalization stats
    # RSL-RL uses RunningMeanStd modules: model.actor_obs_normalizer and model.critic_obs_normalizer
    # They have buffers: running_mean, running_var, count
    
    with torch.no_grad():
        # Ensure shapes match (1, 75)
        mean_tensor = obs_mean.unsqueeze(0).to(device)
        var_tensor = obs_std.pow(2).unsqueeze(0).to(device)
        std_tensor = obs_std.unsqueeze(0).to(device)
        
        # Update Actor Normalizer
        # Attributes are _mean, _var, _std, count
        model.actor_obs_normalizer._mean.copy_(mean_tensor)
        model.actor_obs_normalizer._var.copy_(var_tensor)
        model.actor_obs_normalizer._std.copy_(std_tensor)
        model.actor_obs_normalizer.count.fill_(1.0)
        
        # Update Critic Normalizer
        model.critic_obs_normalizer._mean.copy_(mean_tensor)
        model.critic_obs_normalizer._var.copy_(var_tensor)
        model.critic_obs_normalizer._std.copy_(std_tensor)
        model.critic_obs_normalizer.count.fill_(1.0)

    # Initialize Optimizer
    # We must include ALL parameters so the optimizer state matches what PPO expects
    optimizer = optim.Adam(model.parameters(), lr=args.lr)
    criterion = nn.MSELoss()

    # 5. Training Loop
    best_val_loss = float('inf')
    
    print("Starting training...")
    for epoch in range(args.epochs):
        model.train()
        train_loss = 0.0
        for batch_obs, batch_actions in train_loader:
            batch_obs, batch_actions = batch_obs.to(device), batch_actions.to(device)
            
            optimizer.zero_grad()
            
            # Forward pass through Actor only
            # We manually normalize and pass to actor
            # model.actor_obs_normalizer returns normalized obs
            norm_obs = model.actor_obs_normalizer(batch_obs)
            pred_actions = model.actor(norm_obs)
            
            loss = criterion(pred_actions, batch_actions)
            loss.backward()
            optimizer.step()
            
            train_loss += loss.item()
        
        train_loss /= len(train_loader)

        # Validation
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for batch_obs, batch_actions in val_loader:
                batch_obs, batch_actions = batch_obs.to(device), batch_actions.to(device)
                norm_obs = model.actor_obs_normalizer(batch_obs)
                pred_actions = model.actor(norm_obs)
                loss = criterion(pred_actions, batch_actions)
                val_loss += loss.item()
        
        val_loss /= len(val_loader)

        print(f"Epoch {epoch+1}/{args.epochs} | Train Loss: {train_loss:.6f} | Val Loss: {val_loss:.6f}")

        # Save best model
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            
            # Save full checkpoint compatible with RSL-RL
            checkpoint_dict = {
                "model_state_dict": model.state_dict(),
                "optimizer_state_dict": optimizer.state_dict(),
                "iter": epoch,
                "infos": {"val_loss": val_loss},
                "epoch": epoch,
                "val_loss": val_loss
            }
            
            torch.save(checkpoint_dict, args.output_model)
            print(f"  -> Saved best model to {args.output_model} (RSL-RL Native)")
            
            # Also save as JIT
            try:
                # For JIT, we need to trace the inference path
                # We create a wrapper that mimics how policy is used in inference
                class JITWrapper(nn.Module):
                    def __init__(self, policy):
                        super().__init__()
                        self.policy = policy
                    def forward(self, x):
                        # Mimic act_inference: normalize -> actor
                        # Note: act_inference in rsl_rl usually handles dicts or tensors depending on implementation
                        # But for export we usually want tensor in -> tensor out
                        norm_x = self.policy.actor_obs_normalizer(x)
                        return self.policy.actor(norm_x)
                
                jit_wrapper = JITWrapper(model)
                jit_wrapper.eval()
                dummy_input = torch.randn(1, 75).to(device)
                traced_script_module = torch.jit.trace(jit_wrapper, dummy_input)
                traced_script_module.save(args.output_model.replace(".pt", "_jit.pt"))
            except Exception as e:
                print(f"Warning: Could not save JIT model: {e}")

    print("Training complete.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=str, default="vbc_expert_data.hdf5", help="Path to HDF5 dataset")
    parser.add_argument("--output_model", type=str, default="acc_bc_policy.pt", help="Path to save trained model")
    parser.add_argument("--epochs", type=int, default=50, help="Number of training epochs")
    parser.add_argument("--batch_size", type=int, default=256, help="Batch size")
    parser.add_argument("--lr", type=float, default=2e-4, help="Learning rate")
    
    args = parser.parse_args()
    train_bc(args)
