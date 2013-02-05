package handshaperecognizer;

import java.util.Arrays;

public class Sorter {

	// int keys
	boolean ascending = true;
	
	int partition(int arr[], int left, int right)
	{
	      int i = left, j = right;
	      int tmp;
	      int pivot = arr[(left + right) / 2];
	     
	      while (i <= j) {
	            while (arr[i] < pivot)
	                  i++;
	            while (arr[j] > pivot)
	                  j--;
	            if (i <= j) {
	                  tmp = arr[i];
	                  arr[i] = arr[j];
	                  arr[j] = tmp;
	                  i++;
	                  j--;
	            }
	      };
	     
	      return i;
	}
	 
	public void quickSort(int arr[], int[] indices, int left, int right) {
	      int index = partition(arr, left, right);
	      if (left < index - 1)
	            quickSort(arr, indices, left, index - 1);
	      if (index < right)
	            quickSort(arr, indices, index, right);
	}
	
	
	
	int partition(double arr[], int[] indices, int left, int right)
	{
	      int i = left, j = right;
	      double tmp;
	      double pivot = arr[(left + right) / 2];
	      int indexTmp;
	      
	      while (i <= j) {
	            while (arr[i] < pivot)
	                  i++;
	            while (arr[j] > pivot)
	                  j--;
	            if (i <= j) {
	            	  indexTmp = indices[i];
	            	  indices[i] = indices[j];
	            	  indices[j] = indexTmp;
	                  tmp = arr[i];
	                  arr[i] = arr[j];
	                  arr[j] = tmp;
	                  i++;
	                  j--;
	            }
	      };
	     
	      return i;
	}
	
	public void quickSort(double arr[], int[] indices, int left, int right) {
	      int index = partition(arr, indices, left, right);
	      if (left < index - 1)
	            quickSort(arr, indices, left, index - 1);
	      if (index < right)
	            quickSort(arr, indices, index, right);
	}
	
	public void quickSort(double[] arr, int[] indices) {
		for (int i = 0; i < arr.length; i++) 
			indices[i] = i;
		
		if (!ascending)
			for (int i = 0; i < arr.length; i++)
				arr[i] *= -1;

		quickSort(arr, indices, 0, arr.length - 1);
		
		if (!ascending)
			for (int i = 0; i < arr.length; i++)
				arr[i] *= -1;
	}	

	
	public static void main(String[] args) {
		Sorter sorter = new Sorter();
		sorter.ascending = false;
		double[] nums = new double[5];
		int[] indices = new int[5];
		
		for (int i = 0; i < nums.length; i++) {
			nums[i] = Math.floor(Math.random() * 45 + 55);
		}
		
		System.out.println("" + Arrays.toString(nums));
		sorter.quickSort(nums, indices);
		System.out.println("" + Arrays.toString(nums));
		System.out.println("" + Arrays.toString(indices));
	}
}
