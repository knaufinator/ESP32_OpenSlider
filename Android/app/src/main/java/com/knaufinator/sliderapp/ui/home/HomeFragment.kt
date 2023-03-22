package com.knaufinator.sliderapp.ui.home

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import android.widget.Button
import android.widget.TextView
import android.widget.Toast
import androidx.fragment.app.Fragment
import androidx.lifecycle.ViewModelProvider
import com.knaufinator.sliderapp.databinding.FragmentHomeBinding

class HomeFragment : Fragment() {

    private var _binding: FragmentHomeBinding? = null

    // This property is only valid between onCreateView and
    // onDestroyView.
    private val binding get() = _binding!!

    override fun onCreateView(
            inflater: LayoutInflater,
            container: ViewGroup?,
            savedInstanceState: Bundle?
    ): View {
        val homeViewModel =
                ViewModelProvider(this).get(HomeViewModel::class.java)

        _binding = FragmentHomeBinding.inflate(inflater, container, false)
        val root: View = binding.root

        // Find the buttons by their IDs
        val homeButton =  binding.homeButton
        val runButton =  binding.runButton
        // Set onClick listeners for the buttons
        homeButton.setOnClickListener {
            // Perform the desired action when the Home button is clicked
          //  Toast.makeText(context, "Home button clicked", Toast.LENGTH_SHORT).show()



        }
        // Set onClick listeners for the buttons
        runButton.setOnClickListener {
            // Perform the desired action when the Run button is clicked
            //Toast.makeText(context, "Run button clicked", Toast.LENGTH_SHORT).show()
        }
        return root
    }

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }
}